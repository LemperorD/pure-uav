#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>

int main() {
    int fd = shm_open("pure_uav_shm", O_RDONLY, 0666);
    ftruncate(fd, 0x400000);
    if (fd < 0) {
        perror("shm_open");
        return 1;
    }

    char* p = mmap(NULL, 0x400000, PROT_READ, MAP_SHARED, fd, 0);
    printf("%c %c %c %c\n", p[0], p[1], p[2], p[3]);
    munmap(p, 0x400000);

    return 0;
}