#include "DisplayFace.h"
#include "Config.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h> 

void drawCurrentFace() {
    display.clearDisplay();
    display.setTextSize(4);
    display.setTextColor(SSD1306_WHITE);
    
    int curX = 28; // Tọa độ X mặc định
    const char* face = "";

    // Phân loại biểu cảm cực kì rõ ràng bằng Switch-case, 
    // không sợ bị lỗi nếu sau này đảo lộn thứ tự Enum trong Config.h
    switch (currentState) {
        // Nhóm quan sát, dò tìm
        case STATE_SEARCH_ENEMY:
            curX = 28; face = "-_-"; break;
        case STATE_ATK_LOCK:
            curX = 28; face = "O.O"; break;

        // Nhóm phục hồi, mất phương hướng
        case STATE_REC_RECOVER:
            curX = 28; face = "@_@"; break;

        // Nhóm bế tắc, giằng co
        case STATE_ATK_STALEMATE_BRAKE:
            curX = 22; face = "=_="; break;

        // Nhóm tuyệt vọng, rớt đài, lỗi phần cứng
        case STATE_DEF_LAST_STAND:
            curX = 22; face = "T_T"; break;

        // Nhóm tấn công bạo lực, dồn ép
        case STATE_ATK_STRIKE:
        case STATE_ATK_LIFT:
        case STATE_ATK_DELAY_RUSH:
            curX = 22; face = "MwM"; break;

        // Nhóm phòng thủ, né tránh khẩn cấp (Bao trọn các thủ sườn, thủ đít, né vạch mới thêm)
        case STATE_DEF_ANTI_PUSH: // Cứ giữ tên ở đây dù bạn có bỏ logic hay không
        case STATE_DEF_SIDE_GUARD:
        case STATE_DEF_REAR_GUARD:
        case STATE_DEF_EDGE_AVOID:
        case STATE_DEF_ANTI_LIFT:
            curX = 28; face = ">_<"; break;

        // Các trạng thái mặc định như IDLE, CALIBRATION... đã có chữ ở main.ino lo
        default: 
            return; // Không in mặt gì cả
    }

    // In khuôn mặt ra OLED
    display.setCursor(curX, 0);
    display.print(face);
    display.display();
}

void showLoading() {
    int cx = 64;
    int cy = 16; // Tâm màn hình 128x32

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK); 

    display.setTextWrap(false);

    // PHASE 1: NGÔI SAO XOAY
    for (int frame = 0; frame < 30; frame++) { // Giảm xuống 30 frame để nhịp độ dứt khoát hơn
        display.clearDisplay();
        float angle = frame * 0.2; 
        
        for (int i = 0; i < 4; i++) {
            float a1 = angle + i * (PI / 2.0);
            display.drawLine(cx, cy - 2, cx + cos(a1) * 8, (cy - 2) + sin(a1) * 8, SSD1306_WHITE);
            float a2 = angle + (i * PI / 2.0) + (PI / 4.0);
            display.drawLine(cx, cy - 2, cx + cos(a2) * 4, (cy - 2) + sin(a2) * 4, SSD1306_WHITE);
        }
        
        display.setCursor(34, 24);
        display.print("Loading...");
        display.display();
        delay(20);
    }

    // PHASE 2: SỤP ĐỔ (Collapse)
    for (int r = 16; r >= 4; r -= 4) {
        display.clearDisplay();
        display.fillCircle(cx, cy, r, SSD1306_WHITE);
        display.display();
        delay(30);
    }

    // PHASE 3 & 4: DỊCH CHUYỂN HỐ ĐEN & HIỆN "Complete"
    int bh_x = cx;
    int target_bh_x = 16; 
    int final_text_x = 38; // Căn lại X để "Complete" cân đối với hố đen
    int current_text_x = 128;

    for (int step = 0; step <= 30; step++) {
        display.clearDisplay();

        // Di chuyển hố đen sang trái
        if (bh_x > target_bh_x) bh_x -= 2;
        display.fillCircle(bh_x, cy, 4, SSD1306_WHITE);
        display.drawCircle(bh_x, cy, 6, SSD1306_WHITE);

        // Trượt chữ "Complete" từ phải vào
        if (current_text_x > final_text_x) current_text_x -= 4;
        display.setCursor(current_text_x, cy - 3); // cy-3 giúp chữ nằm ngay giữa trục Y
        display.print("Complete");

        int sweep = 128 - (step * 6);
        if (sweep > current_text_x + 48) { 
            // Chỉ in <<< khi còn cách xa chữ
            display.setCursor(sweep, cy - 3);
            display.print("<<<");
        } else if (sweep > current_text_x + 8) {
            // Khi tới gần, đổi thành 1 tia laser đuổi theo cho mượt
            display.drawFastHLine(sweep, cy, 6, SSD1306_WHITE);
        }

        display.display();
        delay(20);
    }

    // Hiệu ứng Flash Invert (Chớp màn hình)
    display.invertDisplay(true); 
    delay(50); // Giảm nhẹ time chớp để mắt người dùng không bị "mù flash"
    display.invertDisplay(false);

    // PHASE 5: CHỐT KHUNG - "SYSTEM READY"
    display.clearDisplay();
    
    // Chữ có 12 ký tự -> 12 * 6 = 72px. Để giữa màn hình: (128 - 72) / 2 = 28
    display.setCursor(28, cy - 3); 
    display.print("SYSTEM READY"); 

    // Sử dụng drawFastHLine/VLine thay vì drawRect để khung ngắm mảnh, sắc nét và ôm trọn chữ hơn
    // Bracket TRÁI
    display.drawFastHLine(14, cy - 8, 6, SSD1306_WHITE); // Cạnh trên
    display.drawFastVLine(14, cy - 8, 16, SSD1306_WHITE); // Cạnh dọc
    display.drawFastHLine(14, cy + 7, 6, SSD1306_WHITE); // Cạnh dưới
    
    // Bracket PHẢI
    display.drawFastHLine(108, cy - 8, 6, SSD1306_WHITE); // Cạnh trên
    display.drawFastVLine(113, cy - 8, 16, SSD1306_WHITE); // Cạnh dọc
    display.drawFastHLine(108, cy + 7, 6, SSD1306_WHITE); // Cạnh dưới

    display.display();
    delay(1500); 
}