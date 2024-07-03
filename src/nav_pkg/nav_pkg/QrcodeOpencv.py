import cv2
from pyzbar import pyzbar
import os
import numpy as np
from qreader import QReader

class QRCodeReader:
    def __init__(self, directory_path):
        self.directory_path = directory_path
        self._qrreader = QReader()

    def read_qr_codes_from_directory(self):
        for filename in os.listdir(self.directory_path):
            if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp')):
                file_path = os.path.join(self.directory_path, filename)
                self.read_qr_code_from_file(file_path)

    def read_qr_code_from_file(self, file_path):
        image = cv2.imread(file_path)
        if image is None:
            print(f"Could not read file: {file_path}")
            return

        # Converti l'immagine in scala di grigi
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        decoded_text = self._qrreader.detect_and_decode(image=gray)

        # # Trova i QR code nell'immagine
        # decoded_objects = pyzbar.decode(gray)

        # for obj in decoded_objects:
        #     # Decodifica il QR code
        #     qr_code_data = obj.data.decode('utf-8')
        #     print(f"QR Code detected in {file_path}: {qr_code_data}")

        #     # Disegna un rettangolo intorno al QR code
        #     points = obj.polygon
        #     if len(points) == 4:
        #         pts = [tuple(point) for point in points]
        #         pts = np.array(pts, dtype=np.int32)
        #         cv2.polylines(image, [pts], isClosed=True, color=(0, 255, 0), thickness=3)

        # Mostra l'immagine con i QR code individuati
        cv2.imshow('QR Code Reader', image)
        print(decoded_text)
        cv2.waitKey(0)

    def __del__(self):
        cv2.destroyAllWindows()

if __name__ == "__main__":
    directory_path = "./"  # Sostituisci con il percorso della tua directory
    reader = QRCodeReader(directory_path)
    reader.read_qr_codes_from_directory()
