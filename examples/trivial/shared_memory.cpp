#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

int main() {
    const char *shm_name = "/my_shared_memory";
    const int SIZE = 4096;

    // Create and configure shared memory
    int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd, SIZE);

    pid_t pid = fork();
    if (pid == 0) {
        // Child process: Read from shared memory
        char *shared_mem = (char *)mmap(0, SIZE, PROT_READ, MAP_SHARED, shm_fd, 0);
        std::cout << "Child read: " << shared_mem << std::endl;
        munmap(shared_mem, SIZE);
    } else {
        // Parent process: Write to shared memory
        char *shared_mem = (char *)mmap(0, SIZE, PROT_WRITE, MAP_SHARED, shm_fd, 0);
        const char *message = "Hello from parent!";
        sprintf(shared_mem, "%s", message);
        munmap(shared_mem, SIZE);
        wait(NULL);  // Wait for child to finish
    }

    shm_unlink(shm_name);  // Remove shared memory
    return 0;
}
