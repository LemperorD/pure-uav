#!/usr/bin/env python3
"""CLI/config tests. Every subprocess uses --help or --check-config; no devices."""
import pathlib
import subprocess
import sys
import tempfile

binary=pathlib.Path(sys.argv[1]).resolve()
root=pathlib.Path(sys.argv[2]).resolve()
text=(root/'config/app.yaml').read_text()
checks=0

def run(args, expected=0, contains=None, cwd='/tmp'):
    global checks
    result=subprocess.run([str(binary),*args],cwd=cwd,capture_output=True,text=True,timeout=10)
    output=result.stdout+result.stderr
    assert result.returncode==expected,(args,result.returncode,output)
    if contains:
        assert contains in output,(contains,output)
    checks+=1
    return output

run(['--help'],contains='--check-config')
run(['--no-send','--check-config'],contains='odometry_send_enabled: false')
effective=run(['--check-config'],contains='Configuration OK')
run(['--check-config'],contains=str(root/'src/vio/estimator/config/gemini_1920x1080'))
run(['--config'],2,'requires a path')
run(['--unknown'],2,'Unknown argument')
with tempfile.TemporaryDirectory(prefix='app-config-',dir=root/'build') as directory:
    tmp=pathlib.Path(directory)
    def variant(name,old,new,error):
        path=tmp/(name+'.yaml')
        assert old in text
        path.write_text(text.replace(old,new,1))
        run(['--config',str(path),'--check-config'],2,error)
    variant('missing-dir','src/vio/estimator/config/gemini_1920x1080','not-a-real-directory','config_dir')
    variant('bad-loop','main_loop_hz: 50','main_loop_hz: 0','main_loop_hz')
    variant('bad-age','max_age_ms: 150','max_age_ms: -1','max_age_ms')
    variant('nan-wait','heartbeat_wait_s: 5.0','heartbeat_wait_s: .nan','heartbeat_wait_s')
    variant('wrong-resolution','width: 1920','width: 1280','cam0.resolution')
    variant('inconsistent-depth','use_depth: true','use_depth: false','use_depth_init')
    enabled=tmp/'detection.yaml'
    enabled.write_text(text.replace('detection_enabled: false','detection_enabled: true',1))
    if 'detection.npu_available: false' in effective:
        run(['--config',str(enabled),'--check-config'],2,'NPU detection unavailable')
    else:
        run(['--config',str(enabled),'--check-config'],0,'detection_enabled: true')
    variant('guidance','guidance_enabled: false','guidance_enabled: true','guidance_enabled')
    variant('bad-detect-rate','max_hz: 10','max_hz: 0','Invalid detection config')
    variant('bad-detect-class','target_class: -1','target_class: 2','Invalid detection config')
    variant('bad-detect-conf','conf_thr: 0.25','conf_thr: .nan','Invalid detection config')
    variant('bad-detect-age','max_age_ms: 200','max_age_ms: 0','Invalid detection config')
    variant('unknown-setting','main_loop_hz: 50','main_loop_typo: 50','Unknown setting')
    variant('invalid-quality','quality_ok: 90','quality_ok: 101','quality_ok')
    variant('wrong-rate','imu_hz: 200','imu_hz: 100','highres_imu_us')
    path=tmp/'broken.yaml'
    path.write_text('app: [')
    run(['--config',str(path),'--check-config'],2,'Configuration error')
    path=tmp/'driver.yaml'
    path.write_text('communicate:\n  file: /not-a-device\n  baudrate: 1\n')
    run(['--driver-config',str(path),'--check-config'],2,'baudrate')
print(f'PASS: {checks} CLI/config checks from a different working directory; no hardware opened')
