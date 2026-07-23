#!/usr/bin/env python3
"""
Calibração interativa de 3 ROIs (RGB, Latitude, Longitude) a partir de imagem de referência.
Salva resultados em roi_calibration.txt na mesma pasta.
Uso: python3 calibrate_rois.py [caminho_da_imagem]
"""

import cv2
import sys
from pathlib import Path

# === CONFIGURAÇÃO ===
DEFAULT_IMAGE = "resolution_image_live.png"  # imagem na raiz do workspace
ROIS_NOMES = ["RGB (tela principal)", "LATITUDE (texto lat)", "LONGITUDE (texto lon)"]
OUTPUT_FILE = "roi_calibration.txt"

# === ESTADO ===
points = []
rois = []  # lista de (x, y, w, h)
roi_atual = 0
img = None
img_path = None


def click_event(event, x, y, flags, param):
    global points, rois, roi_atual, img

    if event != cv2.EVENT_LBUTTONDOWN:
        return

    points.append((x, y))
    print(f"  Ponto {len(points)}: ({x}, {y})")

    # Quando completou 2 pontos = 1 ROI
    if len(points) % 2 == 0:
        x1, y1 = points[-2]
        x2, y2 = points[-1]
        rx = min(x1, x2)
        ry = min(y1, y2)
        rw = abs(x2 - x1)
        rh = abs(y2 - y1)
        rois.append((rx, ry, rw, rh))

        nome = ROIS_NOMES[roi_atual] if roi_atual < len(ROIS_NOMES) else f"ROI {roi_atual+1}"
        print(f"  → {nome}: x={rx}, y={ry}, w={rw}, h={rh}")

        roi_atual += 1

        # Desenha ROI na imagem para feedback visual
        cv2.rectangle(img, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 2)
        cv2.putText(img, f"{roi_atual}: {nome}", (rx, max(ry - 5, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("Calibracao ROIs", img)

        if roi_atual >= len(ROIS_NOMES):
            print("\n✅ 3 ROIs calibradas! Pressione qualquer tecla para salvar e sair.")
        else:
            prox = ROIS_NOMES[roi_atual]
            print(f"\n→ Próximo: {prox} (clique canto sup.esq + inf.dir)")


def salvar_resultados():
    """Salva roi_calibration.txt no mesmo diretório do script."""
    script_dir = Path(__file__).parent
    out_path = script_dir / OUTPUT_FILE

    with open(out_path, "w") as f:
        f.write(f"# Calibração ROIs - {img_path.name}\n")
        f.write(f"# Resolução: {img.shape[1]}x{img.shape[0]}\n")
        f.write(f"# Formato: nome x y w h\n\n")

        for i, (rx, ry, rw, rh) in enumerate(rois):
            nome = ROIS_NOMES[i].split(" ")[0].lower()  # "rgb", "latitude", "longitude"
            f.write(f"{nome}_roi: {rx} {ry} {rw} {rh}\n")

    print(f"\n💾 Salvo em: {out_path}")
    print("Conteúdo:")
    print(out_path.read_text())


def main():
    global img, img_path

    # Caminho da imagem (argumento ou padrão na raiz do workspace)
    if len(sys.argv) > 1:
        img_path = Path(sys.argv[1])
    else:
        workspace_root = Path(__file__).parent.parent
        img_path = workspace_root / DEFAULT_IMAGE

    if not img_path.exists():
        print(f"❌ Imagem não encontrada: {img_path}")
        print(f"   Uso: python3 {Path(__file__).name} [caminho_para_imagem]")
        print(f"   Padrão: {workspace_root / DEFAULT_IMAGE}")
        sys.exit(1)

    img = cv2.imread(str(img_path))
    if img is None:
        print(f"❌ Falha ao carregar imagem: {img_path}")
        sys.exit(1)

    h, w = img.shape[:2]
    print(f"📷 Imagem: {img_path.name} ({w}x{h})")
    print(f"📁 Saída: {Path(__file__).parent / OUTPUT_FILE}")
    print(f"\n🖱️  Clique 2 pontos por ROI (canto sup.esq + inf.dir)")
    print(f"   Ordem: {', '.join(ROIS_NOMES)}")
    print(f"   Total: {len(ROIS_NOMES)} ROIs = {len(ROIS_NOMES)*2} cliques")
    print(f"   Pressione qualquer tecla após a 3ª ROI para salvar.\n")

    # Janela redimensionável
    cv2.namedWindow("Calibracao ROIs", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Calibracao ROIs", min(1280, w), min(720, h))
    cv2.setMouseCallback("Calibracao ROIs", click_event)
    cv2.imshow("Calibracao ROIs", img)

    print("→ Aguardando cliques... (ESC cancela)")
    key = cv2.waitKey(0)

    if key == 27:  # ESC
        print("\n❌ Cancelado pelo usuário.")
    else:
        if len(rois) == len(ROIS_NOMES):
            salvar_resultados()
        else:
            print(f"\n⚠️  Apenas {len(rois)}/{len(ROIS_NOMES)} ROIs completas. Nada salvo.")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()