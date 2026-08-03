import onnxruntime as ort

# Deve retornar a lista com TensorrtExecutionProvider
print("Disponíveis:", ort.get_available_providers())

# Ao carregar o modelo, força a ordem de prioridade:
session = ort.InferenceSession(
    "/opt/ocr_models/v3_en_rec.onnx",
    providers=['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider']
)
print("Em uso pela sessão:", session.get_providers())
