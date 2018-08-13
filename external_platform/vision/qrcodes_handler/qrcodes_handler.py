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
        bathroom = pyqrcode.create('bathroom')
        bathroom.png('./qr_codes/bathroom.png', scale=20)

        lab_308 = pyqrcode.create('lab_308')
        lab_308.png('./qr_codes/lab_308.png', scale=20)

        lab_303 = pyqrcode.create('lab_303')
        lab_303.png('./qr_codes/lab_303.png', scale=20)

        elevators = pyqrcode.create('lifts')
        elevators.png('./qr_codes/lifts.png', scale=20)
