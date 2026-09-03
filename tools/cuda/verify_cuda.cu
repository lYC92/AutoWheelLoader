#include <cuda_runtime.h>

#include <cstdio>
#include <cstdlib>

#define CUDA_CHECK(call)                                                       \
  do {                                                                         \
    const cudaError_t error = (call);                                           \
    if (error != cudaSuccess) {                                                 \
      std::fprintf(stderr, "%s failed: %s\n", #call, cudaGetErrorString(error)); \
      return EXIT_FAILURE;                                                      \
    }                                                                          \
  } while (false)

__global__ void write_answer(int* value) {
  *value = 42;
}

int main() {
  int device_count = 0;
  CUDA_CHECK(cudaGetDeviceCount(&device_count));
  if (device_count < 1) {
    std::fprintf(stderr, "No CUDA device found.\n");
    return EXIT_FAILURE;
  }

  cudaDeviceProp properties{};
  CUDA_CHECK(cudaGetDeviceProperties(&properties, 0));

  int* device_value = nullptr;
  CUDA_CHECK(cudaMalloc(&device_value, sizeof(int)));
  write_answer<<<1, 1>>>(device_value);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());

  int host_value = 0;
  CUDA_CHECK(cudaMemcpy(&host_value, device_value, sizeof(int), cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaFree(device_value));

  if (host_value != 42) {
    std::fprintf(stderr, "Kernel result mismatch: %d\n", host_value);
    return EXIT_FAILURE;
  }

  std::printf(
      "PASS  CUDA device=%s compute=%d.%d result=%d\n",
      properties.name,
      properties.major,
      properties.minor,
      host_value);
  return EXIT_SUCCESS;
}
