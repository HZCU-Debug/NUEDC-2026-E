#pragma once

namespace storage {

/**
 * @brief 设备校准结果
 */
struct Calibration {
    Calibration()
        : zDown(0.0f),
          x0(0.0f),
          y0(0.0f),
          x1(0.0f),
          y1(0.0f),
          zValid(false),
          paperValid(false) {}

    /** Z 轴下降位置 */
    float zDown;
    /** A4 纸面 (10 cm, 10 cm) 的 X 轴位置 */
    float x0;
    /** A4 纸面 (10 cm, 10 cm) 的 Y 轴位置 */
    float y0;
    /** A4 对角 X 轴位置 */
    float x1;
    /** A4 对角 Y 轴位置 */
    float y1;
    /** Z 轴标定是否有效 */
    bool zValid;
    /** A4 坐标标定是否有效 */
    bool paperValid;
};

/**
 * @brief 从 Flash 读取校准结果
 * @param calibration 接收校准结果
 * @return 存在有效格式的记录时返回 true
 */
bool loadCalibration(Calibration& calibration);

/**
 * @brief 将完整校准结果保存到 Flash
 * @param calibration 待保存的校准结果
 * @return 保存成功时返回 true
 */
bool saveCalibration(const Calibration& calibration);

}
