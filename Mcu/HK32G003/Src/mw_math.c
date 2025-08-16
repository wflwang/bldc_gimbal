/***
 * @file mw_math.c
 * @date 2025-8-13
 * @author MaxwellWang
 * @brief maxwell foc math
*/
/**
 * 
 * Ts => cycle
 * vm = vdc / sqrt3
 * rad = theta * M_PI/180.0f
 *     // 计算 α-β 分量
    float Valpha = Vm * cosf(rad);
    float Vbeta  = Vm * sinf(rad);
    
    // 计算作用时间
    float T1 = (sqrt(3) * Ts / Vdc) * (sqrt(3)/2 * Valpha - 0.5f * Vbeta);
    float T2 = (sqrt(3) * Ts / Vdc) * Vbeta;
    float T0 = Ts - T1 - T2;
    
    // 计算占空比（七段式对称分配）
    float Da = (T1 + T2 + T0/2) / Ts;
    float Db = (T1 - T2 + T0/2) / Ts;
    float Dc = (T0 - T1) / Ts;  // 或 (T0/2 - T1)/Ts + T0/2
    
    printf("θ=%.1f°: Da=%.4f, Db=%.4f, Dc=%.4f\n", theta, Da, Db, Dc);
 * 
 * 
*/
#if 0
// 输入：电角度 theta（度），直流电压 Vdc
void SVPWM_Sector1(float theta, float Vdc) {
    const float Ts = 1.0f;  // 归一化周期
    const float Vm = Vdc / sqrt(3);
    float rad = theta * M_PI / 180.0f;  // 角度转弧度
    
    // 计算 α-β 分量
    float Valpha = Vm * cosf(rad);
    float Vbeta  = Vm * sinf(rad);
    
    // 计算作用时间
    float T1 = (sqrt(3) * Ts / Vdc) * (sqrt(3)/2 * Valpha - 0.5f * Vbeta);
    float T2 = (sqrt(3) * Ts / Vdc) * Vbeta;
    float T0 = Ts - T1 - T2;
    
    // 计算占空比（七段式对称分配）
    float Da = (T1 + T2 + T0/2) / Ts;
    float Db = (T1 - T2 + T0/2) / Ts;
    float Dc = (T0 - T1) / Ts;  // 或 (T0/2 - T1)/Ts + T0/2
    
    printf("θ=%.1f°: Da=%.4f, Db=%.4f, Dc=%.4f\n", theta, Da, Db, Dc);
}
#endif
/***
 * @brief 7段式 svpwm out
 * 0-60 => 000-100-110-111-110-100-000
 * 60-120 => 000-010-110-111-110-010-000
 * 120-180 => 000-010-011-111-011-010-000
 * 180-240 => 000-001-011-111-011-001-000
 * 240-300 => 000-001-101-111-101-001-000
 * 300-360 => 000-100-101-111-101-100-000
 * 
*/
void SVPWM_build(void){

}