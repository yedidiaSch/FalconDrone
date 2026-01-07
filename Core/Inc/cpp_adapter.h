#ifndef CPP_ADAPTER_H
#define CPP_ADAPTER_H

#ifdef __cplusplus
extern "C" {
#endif

// --- Functions callable from C files ---

// 1. System initialization (called once before the loop)
void cpp_init();

// 2. Cyclic tick (called from the Default Task)
void cpp_tick();

#ifdef __cplusplus
}
#endif

#endif // CPP_ADAPTER_H