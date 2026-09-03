# Force Mesa to use WSLg's Direct3D 12 Gallium backend. On this host Mesa
# otherwise selects llvmpipe even though /dev/dxg and the D3D12 driver exist.
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
