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
