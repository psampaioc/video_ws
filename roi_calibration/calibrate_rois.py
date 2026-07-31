#!/usr/bin/env python3
"""
Calibração interativa de 6 ROIs.
Usa a resolução nativa da imagem fornecida.
Salva resultados em roi_calibration.txt na mesma pasta.
"""

import cv2
import sys
from pathlib import Path

# === CONFIGURAÇÃO ===
DEFAULT_IMAGE = "resolution_image_live.png"
ROIS_NOMES = [
    "RGB (tela principal)",
    "LAT (latitude)",
    "LON (longitude)",
    "HDG (heading / bússola)",
    "ALT (altura / altitude)",
    "SPD (velocidade)"
]
PARAM_NAMES = ["roi_rgb", "roi_lat", "roi_lon", "roi_heading", "roi_height", "roi_speed"]
OUTPUT_FILE = "roi_calibration.txt"

# === ESTADO ===
points = []
rois = []
roi_atual = 0
img = None
img_path = None

def click_event(event, x, y, flags, param):
    global points, rois, roi_atual, img

    if event != cv2.EVENT_LBUTTONDOWN:
        return

    points.append((x, y))
    print(f"  Ponto {len(points)}: ({x}, {y})")

    if len(points) % 2 == 0:
        x1, y1 = points[-2]
        x2, y2 = points[-1]
        rx = min(x1, x2)
        ry = min(y1, y2)
        rw = abs(x2 - x1)
        rh = abs(y2 - y1)
        rois.append((rx, ry, rw, rh))

        nome = ROIS_NOMES[roi_atual] if roi_atual < len(ROIS_NOMES) else f"ROI {roi_atual+1}"
        print(f"  -> {nome}: x={rx}, y={ry}, w={rw}, h={rh}")

        roi_atual += 1

        cv2.rectangle(img, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 2)
        cv2.putText(img, f"{roi_atual}: {nome.split(' ')[0]}", (rx, max(ry - 5, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("Calibracao ROIs", img)

        if roi_atual >= len(ROIS_NOMES):
            print(f"\nOK: {len(ROIS_NOMES)} ROIs calibradas! Pressione qualquer tecla para salvar e sair.")
        else:
            prox = ROIS_NOMES[roi_atual]
            print(f"\n-> Próximo: {prox} (clique canto sup.esq + inf.dir)")

def salvar_resultados():
    script_dir = Path(__file__).parent
    out_path = script_dir / OUTPUT_FILE
    h, w = img.shape[:2]

    with open(out_path, "w") as f:
        f.write(f"# Calibração ROIs - Resolucao Nativa: {w}x{h}\n\n")
        for i, (rx, ry, rw, rh) in enumerate(rois):
            param = PARAM_NAMES[i]
            f.write(f"{param}: [{rx}, {ry}, {rw}, {rh}]\n")

    print(f"\nSalvo em: {out_path}")
    print("Conteúdo:")
    print(out_path.read_text())

def main():
    global img, img_path

    if len(sys.argv) > 1:
        img_path = Path(sys.argv[1])
    else:
        workspace_root = Path(__file__).parent.parent
        img_path = workspace_root / DEFAULT_IMAGE

    if not img_path.exists():
        print(f"Erro: Imagem não encontrada: {img_path}")
        sys.exit(1)

    img = cv2.imread(str(img_path))
    if img is None:
        print(f"Erro: Falha ao carregar imagem: {img_path}")
        sys.exit(1)

    # Mantemos a resolução nativa
    h, w = img.shape[:2]

    print(f"Imagem: {img_path.name} (Resolução Nativa: {w}x{h})")
    print(f"Clique 2 pontos por ROI (canto sup.esq + inf.dir)")
    print(f"Ordem: {', '.join([n.split(' ')[0] for n in ROIS_NOMES])}")
    print(f"Total: {len(ROIS_NOMES)} ROIs\n")

    cv2.namedWindow("Calibracao ROIs", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Calibracao ROIs", min(1280, w), min(720, h))
    cv2.setMouseCallback("Calibracao ROIs", click_event)
    cv2.imshow("Calibracao ROIs", img)

    key = cv2.waitKey(0)

    if key == 27:
        print("\nCancelado pelo usuário.")
    else:
        if len(rois) == len(ROIS_NOMES):
            salvar_resultados()
        else:
            print(f"\nAviso: Apenas {len(rois)}/{len(ROIS_NOMES)} ROIs completas. Nada salvo.")

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
