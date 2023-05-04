# This is an example of how make a class and use it
class Pessoa:
    def __init__(self, nome, idade, profissao):
        self.nome = nome
        self.idade = idade
        self.profissao = profissao
    
    def __str__(self):
        return f"Nome: {self.nome}, Idade: {self.idade}, Profissão: {self.profissao}"


pessoas = []

while True:
    nome = input("Digite o nome da pessoa (deixe em branco para encerrar): ")
    if nome == "":
        break
    idade = input("Digite a idade da pessoa: ")
    profissao = input("Digite a profissão da pessoa: ")
    
    pessoa = Pessoa(nome, idade, profissao)
    pessoas.append(pessoa)

pessoas.sort(key=lambda x: x.nome)

for pessoa in pessoas:
    print(pessoa)
