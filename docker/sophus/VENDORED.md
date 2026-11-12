# Sophus (vendored)

- Nguồn: https://github.com/strasdat/Sophus, commit `a621ff2e56c56c839a6c40418d42c3c254424b5c`. Đây là bản không-template mà FAST-LIVO2/KILVO yêu cầu.
- Giấy phép: xem phần đầu các file trong `sophus/` và file `README`.
- Chỉ sửa một chỗ: `sophus/so2.cpp`, hàm khởi tạo `SO2::SO2()`. Thay phép gán `unit_complex_.real() = 1.` (Eigen mới báo lỗi `lvalue required`) bằng `unit_complex_ = std::complex<double>(1., 0.);`.
- Chỉ dùng để build trong `docker/Dockerfile`.
