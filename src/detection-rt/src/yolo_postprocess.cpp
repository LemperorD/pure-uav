#include "yolo_postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/dnn.hpp>
#include <vector>

namespace pure_uav {
namespace detection {

bool postprocess(const NpuOutputView& out, const PostprocessParams& p, FrameResult& r) {
    r.count = 0;
    r.present = false;
    r.u_px = r.v_px = r.w_px = r.h_px = 0.0;
    r.conf = 0.0;
    r.class_id = -1;

    if (!out.data || out.channels != 4 + p.num_classes || out.anchors <= 0 || out.anchors > 100000 || p.img_w <= 0 || p.img_h <= 0 || !std::isfinite(out.letterbox_scale) || out.letterbox_scale <= 0 || !std::isfinite(p.conf_thr) || p.conf_thr <= 0 || p.conf_thr > 1 || !std::isfinite(p.iou_thr) ||
        p.iou_thr <= 0 || p.iou_thr > 1 || p.max_det < 1 || p.max_det > kMaxDetections || p.target_class < -1 || p.target_class >= p.num_classes) {
        return false;
    }

    const int nc = std::min(p.num_classes, out.channels - 4);
    if (nc <= 0) return false;

    const int A = out.anchors;
    const int C = out.channels;
    for (size_t index = 0; index < static_cast<size_t>(A) * C; ++index)
        if (!std::isfinite(out.data[index])) return false;

    std::vector<cv::Rect2d> boxes;
    std::vector<float> scores;
    std::vector<int> class_ids;
    boxes.reserve(static_cast<size_t>(A));
    scores.reserve(boxes.capacity());
    class_ids.reserve(boxes.capacity());

    if (out.channel_major) {
        // [1, 4+nc, A]：每通道一段连续 anchors
        for (int i = 0; i < A; ++i) {
            int best = 0;
            float best_score = out.data[static_cast<size_t>(4) * A + i];
            for (int c = 1; c < nc; ++c) {
                const float v = out.data[static_cast<size_t>(4 + c) * A + i];
                if (v > best_score) {
                    best_score = v;
                    best = c;
                }
            }
            if (best_score > 1 || best_score < 0) return false;
            if (best_score < p.conf_thr || (p.target_class >= 0 && best != p.target_class)) continue;
            const float cx = out.data[i];
            const float cy = out.data[static_cast<size_t>(A) + i];
            const float bw = out.data[static_cast<size_t>(2) * A + i];
            const float bh = out.data[static_cast<size_t>(3) * A + i];
            if (!(bw > 0.0f) || !(bh > 0.0f)) continue;
            boxes.emplace_back(cx - 0.5f * bw, cy - 0.5f * bh, bw, bh);
            scores.push_back(best_score);
            class_ids.push_back(best);
        }
    } else {
        // [1, A, 4+nc]
        for (int i = 0; i < A; ++i) {
            const float* row = out.data + static_cast<size_t>(i) * C;
            int best = 0;
            float best_score = row[4];
            for (int c = 1; c < nc; ++c) {
                if (row[4 + c] > best_score) {
                    best_score = row[4 + c];
                    best = c;
                }
            }
            if (best_score > 1 || best_score < 0) return false;
            if (best_score < p.conf_thr || (p.target_class >= 0 && best != p.target_class)) continue;
            const float cx = row[0];
            const float cy = row[1];
            const float bw = row[2];
            const float bh = row[3];
            if (!(bw > 0.0f) || !(bh > 0.0f)) continue;
            boxes.emplace_back(cx - 0.5f * bw, cy - 0.5f * bh, bw, bh);
            scores.push_back(best_score);
            class_ids.push_back(best);
        }
    }

    if (boxes.empty()) return true;

    // 逐类 NMS（OpenCV 4.5.4 无 NMSBoxesBatched）
    std::vector<DetectionBox> dets;
    dets.reserve(boxes.size());
    const float inv = 1.0f / out.letterbox_scale;
    const float max_x = static_cast<float>(p.img_w - 1);
    const float max_y = static_cast<float>(p.img_h - 1);

    for (int c = 0; c < nc; ++c) {
        std::vector<cv::Rect2d> cls_boxes;
        std::vector<float> cls_scores;
        std::vector<int> cls_indices;
        for (size_t i = 0; i < boxes.size(); ++i) {
            if (class_ids[i] != c) continue;
            cls_boxes.push_back(boxes[i]);
            cls_scores.push_back(scores[i]);
            cls_indices.push_back(static_cast<int>(i));
        }
        if (cls_boxes.empty()) continue;

        std::vector<int> keep;
        cv::dnn::NMSBoxes(cls_boxes, cls_scores, p.conf_thr, p.iou_thr, keep);
        for (int k : keep) {
            const int idx = cls_indices[k];
            const cv::Rect2d& b = boxes[idx];
            float x1 = static_cast<float>(b.x) * inv;
            float y1 = static_cast<float>(b.y) * inv;
            float x2 = static_cast<float>(b.x + b.width) * inv;
            float y2 = static_cast<float>(b.y + b.height) * inv;
            x1 = std::max(0.0f, std::min(x1, max_x));
            y1 = std::max(0.0f, std::min(y1, max_y));
            x2 = std::max(0.0f, std::min(x2, max_x));
            y2 = std::max(0.0f, std::min(y2, max_y));
            if (x2 <= x1 || y2 <= y1) continue;

            DetectionBox d;
            d.x1 = x1;
            d.y1 = y1;
            d.x2 = x2;
            d.y2 = y2;
            d.conf = scores[idx];
            d.class_id = c;
            dets.push_back(d);
        }
    }

    std::sort(dets.begin(), dets.end(), [](const DetectionBox& a, const DetectionBox& b) { return a.conf > b.conf; });

    const int limit = std::min({static_cast<int>(dets.size()), p.max_det, kMaxDetections});
    for (int i = 0; i < limit; ++i) r.boxes[i] = dets[static_cast<size_t>(i)];
    r.count = limit;
    r.present = limit > 0;

    if (r.present) {
        const DetectionBox& d = r.boxes[0];
        r.u_px = 0.5 * (static_cast<double>(d.x1) + static_cast<double>(d.x2));
        r.v_px = 0.5 * (static_cast<double>(d.y1) + static_cast<double>(d.y2));
        r.w_px = static_cast<double>(d.x2 - d.x1);
        r.h_px = static_cast<double>(d.y2 - d.y1);
        r.conf = static_cast<double>(d.conf);
        r.class_id = d.class_id;
    }
    return true;
}

}  // namespace detection
}  // namespace pure_uav
