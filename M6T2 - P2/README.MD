# P2 M6T2

# Parte Prática
O script desenvolvido é capaz de: 
1. Utilizar um apetrecho de interface de vídeo onde está sendo rodado
2. Gravar um vídeo no formato MP4 
3. Utilizando um modelo pré-treinado de rede neural, **`YOLOV8N.pt`** encontrado na raiz do projeto, fazer a detecção de pessoas no vídeo que está sendo gravado frame a frame 
4. Retornar esse vídeo no arquivo **`output.mp4`** que será armazenado na raíz do projeto. 

Para o ponto 1, a seção abaixo de código demonstra como o script utiliza a câmera do aparelho em questão: 
```
# Import do OpenCV 
import cv2 as cv 

# Instância do objeto que fará a leitura da câmera
cam = cv.VideoCapture(0)

# Verifica se a câmera está conectada ou aberta
if not cam.isOpened():
    print("Câmera não encontrada")
    exit()
```

Ponto 2, a seção abaixo de código demonstra como o script faz a instância do objeto de vídeo para o formato MP4:
```
fourcc = cv.VideoWriter_fourcc(*'mp4v')
out = cv.VideoWriter('output.mp4', fourcc, 30.0, (640, 480))  
```  

Ponto 3, a seção abaixo demonstra com a detcção de pessoas é realizada em cada frame do vídeo que está sendo gravado: 
```
from ultralytics import YOLO

# Carregamento do modelo YOLO
model = YOLO('./yolov8n.pt')

while True:
    # Ler os frames onde a rachadura é identificada
    ret, frame = cam.read()

    resultado = model.predict(frame, conf=0.75)

    if ret:
        # Faz um "quadro vermelho" mostrando os resultados de detecção
        frame_resultados = resultado[0].plot()
        cv.imshow("Resultados", frame_resultados)

        """
        A parte faltante deste código está explicada no ponto 4, logo abaixo.
        """

        # Se a tela com a câmera detectar o que a tecla q foi pressionada o vídeo é encerrado
        if cv.waitKey(1) == ord('q'):
            break
```

Ponto 4, a seção abaixo mostra como os frames com resultados são gravados no objeto de vídeo definido,: 
```
# Escreve esses frames no vídeo
        out.write(frame_resultados)
```
