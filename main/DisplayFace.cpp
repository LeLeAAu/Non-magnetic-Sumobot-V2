#include "Config.h"
#include "MotorControl.h"
#include "DisplayFace.h"
#include "SensorTask.h"
#include "FSMTask.h"

void drawCurrentFace() {
    display.clearDisplay();
    display.setTextSize(4);
    display.setTextColor(SSD1306_WHITE);
    
    if (currentState == STATE_ATK_LOCK) display.setCursor(28, 0), display.print("O.O");
    else if (currentState == STATE_SEARCH_ENEMY) display.setCursor(28, 0), display.print("-_-");
    else if (currentState == STATE_REC_RECOVER) display.setCursor(28, 0), display.print("@_@");
    else if (currentState == STATE_ATK_STALEMATE_BRAKE) display.setCursor(22, 0), display.print("=_=");
    else if (currentState == STATE_DEF_LAST_STAND) display.setCursor(22, 0), display.print("T_T");
    else if (currentState == STATE_ATK_FLANK_FRONT || currentState == STATE_ATK_FLANK_SIDE || currentState == STATE_ATK_FLANK_REAR) display.setCursor(22, 0), display.print("OwO");
    else if (currentState == STATE_ATK_STRIKE || currentState == STATE_ATK_LIFT || currentState == STATE_ATK_DELAY_RUSH || currentState == STATE_ATK_FEINT) display.setCursor(22, 0), display.print("MwM");
    else if (currentState >= STATE_DEF_ANTI_PUSH && currentState <= STATE_DEF_ANTI_LIFT) display.setCursor(28, 0), display.print(">_<");
    
    display.display();
}

void showLoading() {
    int cx = 64;
    int cy = 14; // Canh giữa trục y (giả sử màn 128x32 hoặc dùng phần trên của 128x64)

    // ==================================================
    // PHASE 1: NGÔI SAO XOAY (Spinning Star)
    // ==================================================
    for (int frame = 0; frame < 60; frame++) {
        display.clearDisplay();

        float angle = frame * 0.15; // Tốc độ xoay

        // Vẽ ngôi sao 8 cánh (4 dài, 4 ngắn) bằng lượng giác
        for (int i = 0; i < 4; i++) {
            // Cánh dài
            float a1 = angle + i * (PI / 2.0);
            display.drawLine(cx, cy, cx + cos(a1) * 10, cy + sin(a1) * 10, SSD1306_WHITE);
            // Cánh ngắn xen kẽ
            float a2 = angle + (i * PI / 2.0) + (PI / 4.0);
            display.drawLine(cx, cy, cx + cos(a2) * 5, cy + sin(a2) * 5, SSD1306_WHITE);
        }

        // Chữ Loading với dấu chấm động
        int dots = (frame / 8) % 4;
        char loadStr[15] = "Loading";
        for (int d = 0; d < dots; d++) strcat(loadStr, ".");
        
        int textWidth = strlen(loadStr) * 6;
        display.setTextSize(1);
        display.setCursor(64 - (textWidth / 2), 26);
        display.print(loadStr);

        display.display();
        delay(20);
    }

    // ==================================================
    // PHASE 2: PHÌNH TO VÀ SỤP ĐỔ (Swell & Collapse)
    // ==================================================
    // 2.1 Phình to (Swell)
    for (int r = 10; r <= 24; r += 4) {
        display.clearDisplay();
        display.fillCircle(cx, cy, r, SSD1306_WHITE);
        display.display();
        delay(25);
    }

    // 2.2 Sụp đổ thành hố đen (Collapse)
    int bh_radius = 6; // Bán kính hố đen
    for (int r = 24; r >= bh_radius; r -= 6) {
        display.clearDisplay();
        display.fillCircle(cx, cy, r, SSD1306_WHITE); 
        display.drawCircle(cx, cy, r + 4, SSD1306_WHITE); // Vòng event horizon bốc hơi
        display.display();
        delay(30);
    }

    // ==================================================
    // PHASE 3: DỊCH CHUYỂN HỐ ĐEN & HIỆN TEXT (Shift Left)
    // ==================================================
    int bh_x = cx;
    int target_bh_x = 24;
    int text_x = 128; // Giấu text ngoài lề phải

    for (int step = 0; step <= 20; step++) {
        display.clearDisplay();
        
        // Hố đen trượt sang trái
        bh_x = cx - (step * ((cx - target_bh_x) / 20.0));
        display.fillCircle(bh_x, cy, bh_radius, SSD1306_WHITE);
        display.drawCircle(bh_x, cy, bh_radius + 2, SSD1306_WHITE); // Ánh sáng quanh hố đen

        // Text "Complete" lướt từ phải vào
        if (step > 10) {
            text_x = 128 - ((step - 10) * 6); // Trượt dần tới tọa độ x ~ 68
            display.setCursor(text_x, cy - 3);
            display.print("Complete");
        }

        display.display();
        delay(20);
    }

    // ==================================================
    // PHASE 4: INVERT, "<<<" XOÁ HỐ ĐEN & ĐƯA TEXT RA GIỮA
    // ==================================================
    // Flash Invert
    display.invertDisplay(true); 
    delay(100); 
    display.invertDisplay(false);

    int final_text_x = 64 - (8 * 6) / 2; // Căn giữa chữ "Complete" (8 kí tự * 6px / 2 = 24 -> x=40)
    int current_text_x = text_x;

    // Dải quét "<<<" đi từ phải (128) qua trái (-30)
    for (int sweep = 128; sweep >= -30; sweep -= 8) {
        display.clearDisplay();
        
        // Hố đen chỉ tồn tại nếu dải quét chưa đè qua nó
        if (sweep > bh_x) {
            display.fillCircle(bh_x, cy, bh_radius, SSD1306_WHITE);
            display.drawCircle(bh_x, cy, bh_radius + 2, SSD1306_WHITE);
        }

        // Vẽ mũi tên quét
        display.setCursor(sweep, cy - 3);
        display.print("<<<");

        // Đẩy text "Complete" ra giữa đồng bộ với dải quét
        if (sweep < 100 && current_text_x > final_text_x) {
            current_text_x -= 4; // Tốc độ trượt text
            if (current_text_x < final_text_x) current_text_x = final_text_x; // Ép vào tâm
        }
        
        display.setCursor(current_text_x, cy - 3);
        display.print("Complete");

        display.display();
        delay(25);
    }

    // ==================================================
    // PHASE 5: CHỐT KHUNG - CHUẨN BỊ VÀO TRẬN (System Ready)
    // ==================================================
    display.clearDisplay();
    
    // Viền ngắm mục tiêu (Targeting Brackets) cực chiến
    display.drawLine(20, cy - 8, 30, cy - 8, SSD1306_WHITE);
    display.drawLine(20, cy - 8, 20, cy - 2, SSD1306_WHITE);
    
    display.drawLine(108, cy - 8, 98, cy - 8, SSD1306_WHITE);
    display.drawLine(108, cy - 8, 108, cy - 2, SSD1306_WHITE);
    
    display.drawLine(20, cy + 8, 30, cy + 8, SSD1306_WHITE);
    display.drawLine(20, cy + 8, 20, cy + 2, SSD1306_WHITE);
    
    display.drawLine(108, cy + 8, 98, cy + 8, SSD1306_WHITE);
    display.drawLine(108, cy + 8, 108, cy + 2, SSD1306_WHITE);

    display.setCursor(final_text_x, cy - 3);
    display.print("COMPLETE");
    display.display();
    
    delay(1500); // Giữ frame để ngắm
}