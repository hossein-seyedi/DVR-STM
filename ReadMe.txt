حتماً—این متن را می‌توانی مستقیم در پروژه‌ات (README یا کامنت) نگه داری:

---

## ADC4 Regular Ranks → Pin Mapping (Differential)

**ADC4 is configured in Differential mode with 3 regular conversions (Scan mode).**
Each rank reads one differential pair ((V_P - V_N)). The DMA buffer order matches the rank order.

### Rank 1  →  **Phase V (V_V)**

* **Channel:** 5
* **Pins:**

  * **PB12 = V_V_P (positive)**
  * **PB14 = V_V_N (negative)**
* **Measured value:** (V_V = V_{V_P} - V_{V_N})
* **DMA index:** `adc_buf[0]`

### Rank 2  →  **Phase W (V_W)**

* **Channel:** 1
* **Pins:**

  * **PE14 = V_W_P (positive)**
  * **PE15 = V_W_N (negative)**
* **Measured value:** (V_W = V_{W_P} - V_{W_N})
* **DMA index:** `adc_buf[1]`

### Rank 3  →  **Phase U (V_U)**

* **Channel:** 3
* **Pins:**

  * **PB15 = V_U_P (positive)**
  * **PE8  = V_U_N (negative)**
* **Measured value:** (V_U = V_{U_P} - V_{U_N})
* **DMA index:** `adc_buf[2]`

---
