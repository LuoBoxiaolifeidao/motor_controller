#ifndef TACTILE_SENSOR_DRIVER__SENSOR_PARA_HPP_
#define TACTILE_SENSOR_DRIVER__SENSOR_PARA_HPP_

#include <vector>
#include <string>
#include <cstdint>

// 1. 结构体定义 (对应 ctypes.Structure)
// 使用 #pragma pack(push, 1) 确保字节对齐为 1，与 Python 中的 _pack_ = 1 一致
#pragma pack(push, 1)

struct DynamicYddsComTs {
    float nf;
    uint32_t nfCap;
    float tf;
    uint32_t tfCap;
    uint16_t tfDir;
    uint32_t prox;
};

struct DynamicYddsU16Ts {
    uint16_t nf;
    uint16_t tf;
    uint16_t tfDir;
};

#pragma pack(pop)

// 2. 配置类定义
class FingerHeatMap {
public:
    int rows;
    int cols;
    std::string file_path;
    int cap_count;
    std::vector<int> cap_indices;

    FingerHeatMap(int r, int c, std::string path, int count, std::vector<int> indices)
        : rows(r), cols(c), file_path(path), cap_count(count), cap_indices(indices) {}
};

class FingerParamTS {
public:
    int prg;
    int pack_len;
    int sensor_num;
    int touch_num;
    int ydds_num;
    int s_prox_num;
    int m_prox_num;
    int cap_byte;
    int ydds_type;
    int had_err;
    int cali_num;
    std::string name;
    std::string display_type_para;
    std::vector<FingerHeatMap> p_heat_map;

    FingerParamTS(int prg_, int pack_len_, int sensor_num_, int touch_num_, int ydds_num_,
                  int s_prox_num_, int m_prox_num_, int cap_byte_, int ydds_type_,
                  int had_err_, int cali_num_, std::string name_, std::string display_type_para_,
                  std::vector<FingerHeatMap> p_heat_map_)
        : prg(prg_), pack_len(pack_len_), sensor_num(sensor_num_), touch_num(touch_num_),
          ydds_num(ydds_num_), s_prox_num(s_prox_num_), m_prox_num(m_prox_num_),
          cap_byte(cap_byte_), ydds_type(ydds_type_), had_err(had_err_), cali_num(cali_num_),
          name(name_), display_type_para(display_type_para_), p_heat_map(p_heat_map_) {}
};

// 3. 模拟数据定义 (对应文件末尾的静态配置数据)
namespace SensorPara {

static const std::vector<FingerHeatMap> finger2_power_cap_index = {
    FingerHeatMap(16, 8, "TS-F-A/heatMapPara16_8.dat", 7, std::vector<int>(16, 0))
};

static const std::vector<FingerHeatMap> finger17_power_cap_index = {
    FingerHeatMap(6, 7, "TS-T-A/weight6X7X6.dat", 6, std::vector<int>(16, 0)),
    FingerHeatMap(6, 7, "TS-T-A/weight6X7X6.dat", 6, std::vector<int>(16, 0))
};

static const std::vector<FingerHeatMap> finger27_power_cap_index = {
    FingerHeatMap(16, 8, "TS-F-A/heatMapPara16_8.dat", 7, std::vector<int>(16, 0))
};

// 完整的参数列表
static const std::vector<FingerParamTS> finger_params = {
    FingerParamTS(2, 62, 8, 7, 1, 1, 0, 4, 2, 0, 22, "通用手指", "TypeA", finger2_power_cap_index),
    FingerParamTS(17, 78, 16, 13, 2, 2, 1, 3, 4, 1, 22, "两指-大包", "TypeB", finger17_power_cap_index),
    FingerParamTS(27, 66, 16, 15, 1, 1, 0, 3, 4, 1, 22, "通用点阵", "TypeB", finger27_power_cap_index),
    FingerParamTS(44, 60, 14, 13, 1, 1, 0, 3, 4, 1, 22, "通用点阵", "TypeB", finger27_power_cap_index),
    FingerParamTS(50, 36, 6, 5, 1, 1, 0, 3, 4, 1, 22, "通用大拇指", "TypeB", finger27_power_cap_index),
    FingerParamTS(54, 36, 6, 5, 1, 1, 0, 3, 4, 1, 22, "通用小拇指", "TypeB", finger27_power_cap_index),
    FingerParamTS(52, 66, 12, 9, 2, 2, 1, 3, 4, 1, 22, "通用中指", "TypeB", finger27_power_cap_index)
};

} // namespace SensorPara

#endif  // TACTILE_SENSOR_DRIVER__SENSOR_PARA_HPP_