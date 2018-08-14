import pyqrcode
from pyzbar.pyzbar import decode

class QRCodesHandler:
    def detect_QRcodes(self, image):
        detections = []
        qr_codes = decode(image)
        for qr_code in qr_codes:
            bbox = [
                qr_code.rect.left, 
                qr_code.rect.top,
                qr_code.rect.left + qr_code.rect.width,
                qr_code.rect.top + qr_code.rect.height
            ]
            detections.append([bbox, str(qr_code.data.decode('utf-8')), 'QRCODE'])
        return detections


    def generate_QRcodes(self):
        lab306 = pyqrcode.create('hydrant')
        lab306.png('./qr_codes/hydrant.png', scale=20)

        lab308 = pyqrcode.create('lab308')
        lab308.png('./qr_codes/lab308.png', scale=20)

        lab303 = pyqrcode.create('lab303')
        lab303.png('./qr_codes/lab303.png', scale=20)

        lifts = pyqrcode.create('lifts')
        lifts.png('./qr_codes/lifts.png', scale=20)

if __name__ == '__main__':
    handler = QRCodesHandler()
    handler.generate_QRcodes()