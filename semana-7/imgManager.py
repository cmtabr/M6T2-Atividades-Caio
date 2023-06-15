import cv2 as cv
from ultralytics import YOLO
from dotenv import load_dotenv
import os
from supabase import create_client, Client
from datetime import datetime

load_dotenv()

# Inicialização da câmera
cam = cv.VideoCapture(0)

if not cam.isOpened():
    print("Câmera não encontrada")
    exit()

# Carregamento do modelo YOLO
model = YOLO('./model.pt')

# Configurações do Supabase
url = os.environ.get("SUPABASE_URL")
key = os.environ.get("SUPABASE_KEY")
supabase: Client = create_client(url, key)

# Configuração do codec e criação do objeto VideoWriter para salvar o vídeo
fourcc = cv.VideoWriter_fourcc(*'mp4v')
output_folder = datetime.now().strftime("SCAN_%Y_%d_%m_%H_%M")
os.makedirs(output_folder, exist_ok=True)
video_path = os.path.join(output_folder, 'output.mp4')
out = cv.VideoWriter(video_path, fourcc, 30.0, (640, 480))

# Criação da pasta para armazenar as imagens capturadas
image_folder = os.path.join(output_folder, 'images')
os.makedirs(image_folder, exist_ok=True)

frame_counter = 0

# Loop principal
while True:
    # Leitura do próximo frame da câmera
    ret, frame = cam.read()
    
    # Execução da detecção de objetos no frame usando o modelo YOLO
    result = model.predict(frame, conf=0.6)

    if ret:
        # Criação do frame com os resultados da detecção
        frame_result = result[0].plot()

        # Nome do arquivo de imagem
        image_filename = f"image_{frame_counter}.jpg"

        frame_counter += 1

        # Salva o frame no vídeo
        out.write(frame_result)

        # Exibe o frame com os resultados
        cv.imshow("Resultados", frame_result)

        # Verifica se a tecla 'q' foi pressionada para sair do loop
        if cv.waitKey(1) == ord('q'):
            break

        # Salva o frame como uma imagem
        image_path = os.path.join(image_folder, image_filename)
        cv.imwrite(image_path, frame_result)

        # Faz upload da imagem para o Supabase
        with open(image_path, 'rb') as f:
            try:
                res = supabase.storage.from_('images').upload(
                    f"{output_folder}/{image_filename}",
                    f.read(),
                    {"contentType": "image/jpeg"}
                )
                if res.status_code != 200:
                    print('Erro ao fazer upload da imagem:', res.text)
            except Exception as e:
                print('Erro ao fazer upload da imagem:', str(e))

        # Remove o arquivo de imagem
        os.remove(image_path)

# Libera a câmera e fecha o objeto VideoWriter
cam.release()
out.release()

# Faz upload do vídeo para o Supabase
with open(video_path, 'rb') as f:
    try:
        res = supabase.storage.from_('videos').upload(
            f"{output_folder}/output.mp4",
            f.read(),
            {"contentType": "video/mp4"}
        )
        if res.status_code != 200:
            print('Erro ao fazer upload do vídeo:', res.text)
    except Exception as e:
        print('Erro ao fazer upload do vídeo:', str(e))

# Remove o arquivo de vídeo
os.remove(video_path)

# Remove a pasta de imagens
os.rmdir(image_folder)

# Remove a pasta de saída
os.rmdir(output_folder)

# Fecha todas as janelas abertas
cv.destroyAllWindows()
