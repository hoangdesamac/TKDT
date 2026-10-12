# ESP-NOW / Raspberry Pi USB bridge

Firmware này biến ESP32 thành cầu nối hai chiều:

`Raspberry Pi -- USB Serial 115200 -- ESP32 -- ESP-NOW -- CYD`

## Cấu hình

1. Lấy địa chỉ STA MAC của CYD bằng `WiFi.macAddress()`.
2. Thay `CYD_MAC_ADDRESS` trong `src/main.cpp` (hiện tại là
   `F4:2D:C9:7A:6A:D4`) và giữ hai bo ở cùng `ESPNOW_CHANNEL=1`.
3. Chép `include/cyd_protocol.h` vào project CYD và dùng đúng
   `cyd::SystemPacket`. Packet của hai bo bắt buộc có cùng layout 16 byte.

ESP32 chỉ nhận lệnh từ MAC CYD đã cấu hình và gửi unicast về địa chỉ đó.
MAC STA của ESP32 được in trong dòng `STATUS`; dùng địa chỉ này làm
`conveyorAddress` trong project CYD.

Packet khớp code CYD: `type` (byte 0), `isRunning` (byte 1), `phase`
(byte 2), `scanId` (byte 3–6), `resultType` (byte 7), `brix` (byte 8–11),
`moisture` (byte 12–15). Mã lệnh: TOGGLE=1, SCAN_REQ=2, SCAN_RES=3,
STATUS=4; FAULT=9.

## Luồng CYD

- CYD gửi `PKT_CMD_TOGGLE` với `isRunning=true/false`.
- ESP32 38-pin nhận lệnh trong callback, sau đó thực thi tại `loop()`.
- ESP32 trả `PKT_STATUS` ngay sau lệnh và gửi heartbeat mỗi giây.
- Mỗi lần state machine đổi pha, gọi `setSystemPhase(...)`; trạng thái mới sẽ
  được gửi sang CYD.

START cho phép luồng băng tải hoạt động; STOP tắt relay và đưa cả hai servo về chờ.
Relay GPIO 26 kích LOW (ON), HIGH (OFF), được đặt OFF ngay khi vào `setup()`.
Lệnh START/STOP từ USB cũng điều khiển luồng này. START lặp trong lúc chờ Pi
không được bật lại băng tải hoặc tạo thêm yêu cầu camera.

Khi khởi động, Serial in `RELAY_INIT PIN=26 OFF_LEVEL=1 ON_LEVEL=0`.
Khi băng tải chạy, Serial in `RELAY_CMD PIN=26 LEVEL=0 REQUEST=ON`;
STOP in `LEVEL=1 REQUEST=OFF`. Đây là mức GPIO được yêu cầu, không phải
phản hồi điện áp hoặc xác nhận tiếp điểm relay đã đóng.

Callback chỉ kiểm tra MAC/độ dài/mã lệnh/byte bool và đưa gói vào queue.
`cydLink.poll()` gọi handler trong `loop()`. `TX_OK` là ACK ở tầng radio,
không xác nhận CYD đã vẽ màn hình; `RX_DROP` đếm gói bị bỏ khi queue đầy.

Phần khai báo bên CYD nên dùng trực tiếp header chung:

```cpp
#include "cyd_protocol.h"

using cyd::SystemPacket;
using cyd::PKT_CMD_TOGGLE;
using cyd::PKT_STATUS;

SystemPacket txPacket{};
SystemPacket rxPacket{};
```

## Giao thức COM với Raspberry Pi

- `STATUS`: xem trạng thái controller và ESP-NOW.
- `SYNC`: yêu cầu gửi trạng thái hiện tại sang CYD ngay lập tức.
- `START`, `STOP` hoặc `RUN 1`, `RUN 0`: thử bật/tắt hệ thống từ Pi.
- `0` hoặc `1`: kết quả của trạm đang chờ Pi (camera hoặc buồng tối).
- Pha được cập nhật tự động theo luồng IR1/IR2, không dùng lệnh `PHASE` nữa.

## IR1 và camera

IR1 dùng GPIO 32, phát hiện vật ở mức LOW, chống dội 30 ms. Khi hệ thống
đang chạy và IR1 có vật:

1. Dừng băng tải, đợi 200 ms cho ảnh ổn định.
2. Gửi đúng một dòng `CAMERA\n` qua USB Serial 115200.
3. Chương trình của bạn trên Pi nhận dòng này, xử lý ảnh rồi gửi `0\n`
   (khuyết tật) hoặc `1\n` (đạt kiểm tra ngoại quan). ESP32 không chạy mô hình.
4. Kết quả `0`: servo1 gạt 20°, đợi 300 ms, bật băng tải cho tới khi IR1
   không thấy vật. Sau đó dừng băng tải, trả servo về 65°, đợi 300 ms rồi
   chạy tiếp để nhận trái mới.
5. Kết quả `1`: giữ servo1 ở 65°, chạy băng tải đưa xoài vào buồng tối.
   Không gửi CAMERA lần nữa cho tới khi xử lý xong trái đó tại IR2.

Servo1 dùng PCA9685 địa chỉ `0x40`, kênh 0, SDA=21/SCL=22, PWM 50 Hz.
Hàm `setServoAngle()` dùng `setPWM(channel, 0, pulse)` với dải đếm PWM
150–600 theo cấu hình đã thử thực tế; không dùng `writeMicroseconds()`.
Servo về 65° khi khởi động hoặc STOP. Thời gian chờ servo dùng `millis()`;
không chèn `delay(1000)` để STOP từ CYD vẫn được xử lý.
Nguồn servo phải được cấp riêng theo phần cứng; PCA9685 có phản hồi I2C
không có nghĩa là đã có nguồn cho motor servo. Góc và mức phát hiện IR1
được cấu hình ở đầu `src/main.cpp`.

Quá 30 giây không có kết quả Pi hoặc quá 10 giây vẫn chưa rời IR1 thì
controller vào FAULT và dừng băng tải. Sau khi xử lý nguyên nhân, nhấn START
trên CYD hoặc gửi START qua USB để chạy lại. Nếu báo `PCA9685_NOT_READY`,
kiểm tra kết nối rồi khởi động lại ESP32. Các thời gian nằm trong
`include/camera_sorter.h`.

## IR2, buồng tối và servo2

IR2 dùng GPIO 33, mức phát hiện LOW, chống dội 30 ms. Luồng xử lý tuần tự
một trái xoài; không cấp thêm trái vào IR1 khi trái trước chưa xử lý xong ở IR2.

1. Xoài qua camera với kết quả `1` được băng tải đưa đến IR2.
2. IR2 phát hiện vật: tắt relay trước, rồi gửi `IR2 BLOCKED\n` qua USB/COM.
   Pi kích mô hình buồng tối khi nhận đúng dòng này trong một lượt đang chạy.
3. Băng tải giữ dừng trong lúc chờ Pi trả `0\n` (chua) hoặc `1\n` (ngọt).
4. Chua (`0`): servo2 gạt, đợi 300 ms rồi bật băng tải. Giữ servo2 gạt đến khi
   IR2 hết thấy vật. Sau đó dừng băng tải, trả servo2 về chờ, đợi 300 ms rồi chạy tiếp.
5. Ngọt (`1`): servo2 giữ vị trí chờ, băng tải chạy đưa xoài tới cuối và tự rơi.
   Khi IR2 hết thấy vật, controller sẵn sàng nhận trái tiếp theo; không có cảm biến
   xác nhận xoài đã rơi ở cuối băng tải.
6. Mỗi lần IR2 đổi trạng thái ổn định, báo `IR2 BLOCKED` hoặc `IR2 CLEAR`.
   `IR2 CLEAR` chỉ là trạng thái cảm biến, không yêu cầu Pi gửi thêm kết quả.
   START khi vật đã ở IR2 sẽ gửi lại `IR2 BLOCKED` để bắt đầu lượt đo mới.

Servo2 dùng PCA9685 **kênh 1**, cùng hàm `setServoAngle()` với servo1.
`SERVO2_HOME_ANGLE=65` và `SERVO2_GATE_ANGLE=20` trong `src/main.cpp` là góc
tạm để bạn tự hiệu chỉnh cơ cấu thực tế, chưa được kiểm chứng trên servo2.
STOP và FAULT đưa cả hai servo về chờ. Không dùng delay chặn khi gạt servo.

Quá 30 giây không tới IR2 sau khi rời IR1, quá 30 giây không có kết quả buồng tối,
hoặc quá 10 giây không rời IR2 sau khi chạy băng tải thì vào FAULT và tắt relay.
Chỉnh các thời gian `kIr2ArrivalTimeoutMs`, `kPiTimeoutMs`, `kIr2ClearTimeoutMs`
trong `include/camera_sorter.h` nếu cơ cấu thực tế cần lâu hơn.

Gói ESP-NOW vẫn giữ 16 byte và mã pha cũ. `WAIT_PI` dùng chung cho cả hai
trạm; giao diện CYD đang ghi cố định IR1 ở pha này cần đổi chữ nếu muốn phân biệt.
`resultType`: 0=chưa có phân loại cuối, 1=ngọt, 2=chua, 3=khuyết tật ngoại quan.

`0` và `1` phải kết thúc bằng xuống dòng. Các dòng log khác có thể xen kẽ;
phía Pi chỉ kích mô hình tương ứng khi toàn bộ dòng bằng `CAMERA` hoặc
`IR2 BLOCKED` trong lượt đang chạy. Không mở Serial
Monitor cùng lúc với chương trình Pi đang giữ cùng cổng COM.

Protocol kết quả chỉ có 0/1, không có mã lượt: chương trình Pi phải gửi một
kết quả cho mỗi yêu cầu CAMERA hoặc đo IR2, không gửi lại kết quả của lượt đã bị STOP/timeout
sau khi hệ thống đã bắt đầu lượt mới.

IR1/IR2 hết thấy vật được dùng làm điều kiện kết thúc gạt tương ứng; cảm biến
không đo trực tiếp việc xoài đã rơi vào rổ. Các trạng thái cảm biến vẫn có thể được
in khi STOP; chương trình Pi không nên chạy mô hình khi hệ thống đang dừng.

Trên Raspberry Pi:

```bash
python3 -m pip install pyserial
python3 tools/pi_com.py /dev/ttyUSB0
```

Nếu bo hiện thành USB CDC thì cổng thường là `/dev/ttyACM0`.
