from threading import Thread
from time import sleep
from wpilib import AddressableLED, Color8Bit


class LightStrip:
    def __init__(self, port: int, length=60) -> None:
        self.light_strip_length = length

        self.light_strip_buffer = [AddressableLED.LEDData() for _ in range(length)]
        self.light_strip = AddressableLED(port)
        self.light_strip.setLength(length)
        self.light_strip.setData(self.light_strip_buffer)
        self.light_strip.start()

        self.rainbow_first_pixel_hue = 0

    def __getitem__(self, index: int) -> AddressableLED.LEDData:
        return self.light_strip_buffer[index]

    def update(self) -> None:
        self.light_strip.setData(self.light_strip_buffer)

    def stop(self) -> None:
        self.light_strip.stop()

    def setColor(self, color: Color8Bit) -> None:
        for led in self.light_strip_buffer:
            led.setLED(color)

    def setRGB(self, r: int, g: int, b: int) -> None:
        for led in self.light_strip_buffer:
            led.setRGB(r, g, b)

    def setHSV(self, h: int, s: int, v: int) -> None:
        for led in self.light_strip_buffer:
            led.setHSV(h, s, v)
        # for i in range(len(self.light_strip_buffer)):
        #     self.light_strip_buffer[i].setHSV(h, s, v)

    def setRainbow(self, move=True) -> None:
        if not move:
            self.rainbow_first_pixel_hue = 0

        for i in range(len(self.light_strip_buffer)):
            pixel_hue = int(
                (self.rainbow_first_pixel_hue + (i * 180 / self.light_strip_length))
                % 180
            )
            self.light_strip_buffer[i].setHSV(pixel_hue, 255, 255)

        if move:
            self.rainbow_first_pixel_hue += 3
            self.rainbow_first_pixel_hue %= 180

    def _setRainbowSlow(self) -> None:
        for i in range(len(self.light_strip_buffer)):
            pixel_hue = int(
                (self.rainbow_first_pixel_hue + (i * 180 / self.light_strip_length))
                % 180
            )
            self.light_strip_buffer[i].setHSV(pixel_hue, 255, 255)
            sleep(0.1)

    def setRainbowSlow(self) -> None:
        Thread(target=self._setRainbowSlow).start()
