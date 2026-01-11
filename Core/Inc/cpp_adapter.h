#ifndef CPP_ADAPTER_H
#define CPP_ADAPTER_H

#ifdef __cplusplus
extern "C" {
#endif

// --- Functions callable from C files ---


void imuInit();
void imuTick();



#ifdef __cplusplus
}
#endif

#endif // CPP_ADAPTER_H