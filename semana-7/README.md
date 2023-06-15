# Semana 7 
Como solicitado no autoestudo "Aplicação de OpenCV com Yolo", abaixo vê-se o vídeo relativo ao armazenamento de imagens e vídeos.


https://github.com/cmtabr/M6T2-Atividades-Caio/assets/99201276/4b457204-a67e-4d12-b993-7439d2e3cd7a



# Objetivo: 
Atestar a capacidade do aluno de realizar a detecção de imagens através da visão computacional, utilizando modelos de rede neural e fazer o armazenamento adequado das imagens, neste caso utilizando um bucket da plataforma [SUPABASE](https://supabase.com/).

# Solução: 
Para realizar esta tarefa, foi utilizado o [OpenCV](https://opencv.org/), para realizar a gravação de vídeos/detecção de imagens. Além disso, utilizando os métodos do Supabase, é realizado o envio das imagens e do vídeo para um bucket, sem a necessidade de um servidor para fazer o envio destes dados.

# Métodos Utilizados: 
``` 
with open(source, 'rb+') as f:
  res = supabase.storage().from_('bucket_name').upload(destination, os.path.abspath(source))
```

# Conclusão: 
A atividade requisita fora realizada com o arquivo **`imgManager.py`** presente neste repositório. A partir deste script, o usuário pode realizar a gravação de vídeos através de uma câmera no próprio computador ou em outro dispositivo, além disto o código desenvolvido faz o upload destas imagens em um bucket de forma que seja possível acessar os conteúdos de cada gravação em quaisquer outros dispositivos. 
