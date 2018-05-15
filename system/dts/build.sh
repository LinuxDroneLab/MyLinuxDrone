# non serve inizializzare i2c2 via dts.
# lo faccio direttamente dalla pru
# serve invece togliere il pin-muxing di P9_20 e P9_19 da arm335x-bone-common.dts.
# in proposito verificare se possibile farlo con un overlay
dtc -O dtb -o PRU-I2C2-00A0.dtbo -b 0 -@ PRU-I2C2-00A0.dts

