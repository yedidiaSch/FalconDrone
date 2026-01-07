#include "cpp_adapter.h"
#include "main.h"      // Access to HAL
#include "gpio.h"      // Access to pin definitions (LD2)

// --- Defining the main class (the basic "Brain") ---
class MainApp {
public:
    // Constructor - good place to initialize variables
    MainApp() {
        // Nothing to initialize yet, but here will be PID values etc. in the future
    }

    void init() {
        // Here we will initialize sensors in the future
    }

    void run() {
        // This is the operation that will run repeatedly
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
};

// --- Creating the object (Global Instance) ---
// The object is created in static memory as soon as the system starts.
// No dynamic allocations!
static MainApp myApp;


// --- Implementation of Adapter functions ---

void cpp_init() {
    myApp.init();
}

void cpp_tick() {
    myApp.run();
}