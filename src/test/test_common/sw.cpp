#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

int main(int argc, char* argv[]) {
    int fd = shm_open("pure_uav_shm", O_CREAT | O_RDWR, 0666);
    if (fd < 0) {
        perror("shm_open");
        return 1;
    }
    if (ftruncate(fd, 0x400000) != 0) {
        perror("ftruncate");
        close(fd);
        return 1;
    }

    char* p = static_cast<char*>(mmap(NULL, 0x400000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
    if (p == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return 1;
    }
    memset(p, 'A', 0x400000);
    munmap(p, 0x400000);
    close(fd);

    return 0;
}
