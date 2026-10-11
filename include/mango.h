#pragma once

enum class MangoType {
  NONE, // Chưa xác định / Không có xoài trên trạm
  TOP1, // Xoài Loại 1 (Chất lượng cao nhất -> Đi thẳng xuống cuối)
  TOP2, // Xoài Loại 2 (Chất lượng trung bình -> Gạt qua Servo 2)
  TOP3  // Xoài Loại 3 / Xoài Hỏng (Chất lượng kém -> Gạt qua Servo 1)
};

struct Mango {
  int DM = 0;                       // Độ khô (Dry Matter) đọc từ buồng tối
  bool detected = false;            // Trạng thái phát hiện xoài
  MangoType type = MangoType::NONE; // Phân loại xoài
};

// 1. DỮ LIỆU ĐIỀU KHIỂN & CẤU HÌNH (Gửi từ CYD -> Main ESP32)
struct ControlData {
  bool isRunning;
  bool resetcounters;
};

// 2. DỮ LIỆU TRẠNG THÁI & SỐ LIỆU (Gửi từ Main ESP32 -> CYD)
struct StatusData {
  bool isRunning;
  int countTOP1;
  int countTOP2;
  int countTOP3;
  bool isServo1;
  bool isServo2;
  bool isIR1;
  bool isIR2;
  bool isIR3;

  // --- BÁO LỖI / TRẠNG THÁI GỬI NGƯỢC VỀ CYD ---
  bool isError;    // Báo lỗi kẹt băng tải / cảm biến
};