import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision import datasets, transforms
import matplotlib.pyplot as plt
import numpy as np
import os


# --- 1. DEFINICIÓN DE LA RED (Debe ser idéntica a la del entrenamiento) ---
class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = nn.Conv2d(1, 32, 3, 1)
        self.conv2 = nn.Conv2d(32, 64, 3, 1)
        self.dropout1 = nn.Dropout(0.25)
        self.dropout2 = nn.Dropout(0.5)
        self.fc1 = nn.Linear(9216, 128)
        self.fc2 = nn.Linear(128, 10)

    def forward(self, x):
        x = self.conv1(x)
        x = F.relu(x)
        x = self.conv2(x)
        x = F.relu(x)
        x = F.max_pool2d(x, 2)
        x = self.dropout1(x)
        x = torch.flatten(x, 1)
        x = self.fc1(x)
        x = F.relu(x)
        x = self.dropout2(x)
        x = self.fc2(x)
        output = F.log_softmax(x, dim=1)
        return output


def test():
    # Configuración
    BATCH_SIZE = 1000
    device = torch.device("cpu")
    model_path = "my_network.pt"

    # 2. Cargar Dataset de TEST (No usamos el de train para no hacer trampas)
    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.1307,), (0.3081,))
    ])

    # download=True descargará los datos si no están en la carpeta ./data
    test_dataset = datasets.MNIST('./data', train=False, download=True, transform=transform)
    test_loader = torch.utils.data.DataLoader(test_dataset, batch_size=BATCH_SIZE, shuffle=True)

    # 3. Cargar el Modelo Entrenado
    if not os.path.exists(model_path):
        print(f"❌ ERROR: No encuentro el archivo '{model_path}' en esta carpeta.")
        return

    model = Net().to(device)
    try:
        model.load_state_dict(torch.load(model_path, map_location=device))
        model.eval()
        print(f"✅ Modelo '{model_path}' cargado correctamente.")
    except Exception as e:
        print(f"❌ Error cargando pesos: {e}")
        return

    # 4. Evaluación Matemática (Accuracy)
    print("\nEvaluando precisión en 10,000 imágenes de prueba...")
    correct = 0
    total = 0

    with torch.no_grad():
        for data, target in test_loader:
            data, target = data.to(device), target.to(device)
            output = model(data)
            # Obtener el índice con mayor probabilidad
            pred = output.argmax(dim=1, keepdim=True)
            correct += pred.eq(target.view_as(pred)).sum().item()
            total += target.size(0)

    accuracy = 100. * correct / total
    print(f"\n📊 RESULTADO FINAL:")
    print(f"   Aciertos: {correct}/{total}")
    print(f"   Precisión: {accuracy:.2f}%")

    if accuracy < 90:
        print("⚠️  ALERTA: Tu modelo es bastante malo. Debería dar >98%. Reentrena con más épocas.")
    else:
        print("🚀 El modelo funciona de lujo con números manuscritos.")

    # 5. Evaluación Visual (Ver predicciones)
    print("\nMostrando ejemplos visuales...")
    visualize_predictions(model, test_loader)


def visualize_predictions(model, loader):
    # Cogemos un batch
    data, target = next(iter(loader))

    with torch.no_grad():
        output = model(data)
        preds = output.argmax(dim=1, keepdim=True)

    # Convertir a numpy para pintar
    images = data.numpy()
    preds = preds.numpy()
    targets = target.numpy()

    # Pintar un mosaico de 5x5
    fig = plt.figure(figsize=(10, 10))
    for i in range(25):
        ax = fig.add_subplot(5, 5, i + 1)
        # La imagen viene normalizada, se verá un poco rara de contraste pero se entiende
        img = images[i][0]

        ax.imshow(img, cmap='gray')

        predicted = preds[i][0]
        actual = targets[i]

        # Verde si acierta, Rojo si falla
        color = 'green' if predicted == actual else 'red'

        ax.set_title(f"P:{predicted} (Real:{actual})", color=color)
        ax.axis('off')

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    test()