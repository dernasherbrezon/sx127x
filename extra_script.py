Import('env')

if env.get("PIOPLATFORM") == "ststm32":
    env.Replace(SRC_FILTER=["+<*>", "-<*_spi*>", "+<*_stm32_spi.c>"])

if env.get("PIOPLATFORM") == "gd32":
    env.Replace(SRC_FILTER=["+<*>", "-<*_spi*>", "+<*_gd32_spi.c>"])

if env.get("PIOPLATFORM") == "espressif32":
    env.Replace(SRC_FILTER=["+<*>", "-<*_spi*>", "+<*_esp_spi.c>"])
