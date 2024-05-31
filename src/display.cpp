#include "main.h"
#include <string>

static lv_res_t resetOdom(lv_obj_t *btn) {
    uint8_t id = lv_obj_get_free_num(btn);
    if (id == 0) {
    IMU inertial (6);
    inertial.reset();
    position.setPoint(0, 0);
    }
    return LV_RES_OK;
}
static const char *autonMap[] = {"far safe", "near safe", "\n", "near rush", "near rush elims", "\n", "skills", "skills (no)", "\n", "nothing", ""};

static lv_res_t switchAuton(lv_obj_t *btn, const char *txt) {
    if (txt == autonMap[0]) autonnum = 1;
    else if (txt == autonMap[1]) autonnum = 2;
    else if (txt == autonMap[3]) autonnum = 3;
    else if (txt == autonMap[4]) autonnum = 4;
    else if (txt == autonMap[6]) autonnum = 5;
    else if (txt == autonMap[7]) autonnum = 6;
    else if (txt == autonMap[9]) autonnum = 7;

    return LV_RES_OK;
}

void displayControl() {
    lv_obj_t *tabview;
    tabview = lv_tabview_create(lv_scr_act(), NULL);

    lv_obj_t *tab1 = lv_tabview_add_tab(tabview, "Odometry");
    lv_obj_t *tab2 = lv_tabview_add_tab(tabview, "Selector");

    //odom

    lv_obj_t *odomLabel = lv_label_create(tab1, NULL);
    lv_obj_align(odomLabel, NULL, LV_ALIGN_IN_LEFT_MID, 0, 0);
    std::string odomText = "X: " + std::to_string(position.getX()) + "\n" 
    + "Y: " + std::to_string(position.getY()) + "\n"
    + "Bearing: " + std::to_string(radToDeg(bearing));
    lv_label_set_text(odomLabel, odomText.c_str());

    lv_obj_t *odomButton = lv_btn_create(tab1, NULL);
    lv_obj_set_size(odomButton, 180, 180);
    lv_obj_align(odomButton, NULL, LV_ALIGN_IN_RIGHT_MID, 0, -20);
    lv_btn_set_action(odomButton, LV_BTN_ACTION_CLICK, resetOdom);
    lv_obj_t *odomButtonLabel = lv_label_create(odomButton, NULL);
    lv_label_set_text(odomButtonLabel, "Calibrate");

    //selector

    lv_obj_t *autonMatrix = lv_btnm_create(tab2, NULL);
    lv_btnm_set_map(autonMatrix, autonMap);
    lv_btnm_set_action(autonMatrix, switchAuton);
    lv_obj_set_size(autonMatrix, 360, 160);
    lv_obj_align(autonMatrix, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -20);
    lv_btnm_set_toggle(autonMatrix, true, 6);

    //loop

    while(true) {
        
        odomText = "X: " + std::to_string(position.getX()) + "\n" 
        + "Y: " + std::to_string(position.getY()) + "\n"
        + "Bearing: " + std::to_string(radToDeg(bearing));
        lv_label_set_text(odomLabel, odomText.c_str());

        delay(10);
    }

}