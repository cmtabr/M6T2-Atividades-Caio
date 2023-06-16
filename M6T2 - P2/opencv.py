# Import do OpenCV e do YOLO
import cv2 as cv 
from ultralytics import YOLO

# Instância do objeto que fará a leitura da câmera
cam = cv.VideoCapture(0)

# Carregamento do modelo YOLO
model = YOLO('./yolov8n.pt')

# Verifica se a câmera está conectada ou aberta
if not cam.isOpened():
    print("Câmera não encontrada")
    exit()

fourcc = cv.VideoWriter_fourcc(*'mp4v')
out = cv.VideoWriter('output.mp4', fourcc, 30.0, (640, 480))  

while True:
    # Ler os frames onde a rachadura é identificada
    ret, frame = cam.read()

    resultado = model.predict(frame, conf=0.75)

    if ret:
        # Faz um "quadro vermelho" mostrando os resultados de detecção
        frame_resultados = resultado[0].plot()
        cv.imshow("Resultados", frame_resultados)

        # Escreve esses frames no vídeo
        out.write(frame_resultados)

        # Se a tela com a câmera detectar o que a tecla q foi pressionada o vídeo é encerrado
        if cv.waitKey(1) == ord('q'):
            break

# Retorna os outputs da câmera e vídeo 
cam.release()
out.release()
# Fecha a tela com a câmera
cv.destroyAllWindows()