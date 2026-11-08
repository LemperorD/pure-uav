#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

int main() {
    int fd = shm_open("pure_uav_shm", O_RDONLY, 0666);
    if (fd < 0) {
        perror("shm_open");
        return 1;
    }
    struct stat info {};
    if (fstat(fd, &info) != 0 || info.st_size < 0x400000) {
        fprintf(stderr, "Shared-memory region is missing or smaller than expected\n");
        close(fd);
        return 1;
    }

    char* p = static_cast<char*>(mmap(NULL, 0x400000, PROT_READ, MAP_SHARED, fd, 0));
    if (p == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return 1;
    }
    printf("%c %c %c %c\n", p[0], p[1], p[2], p[3]);
    munmap(p, 0x400000);
    close(fd);

    return 0;
}
