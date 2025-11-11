import pickle
import yaml
import numpy as np
import sys

from virtualmocap.vision.linear_projection import *
from virtualmocap.vision.lens_distortion import *
from virtualmocap.vision.image_noise import *
from virtualmocap.vision.camera import Camera

CALIB_FOLDER = '25-10-24-17-47-24'
ARQUIVO_PKL_ENTRADA = f'assets/calibration/{CALIB_FOLDER}/E45F019003BA.pkl' 
ARQUIVO_YAML_SAIDA = 'src/estimate_markers_poses/config/camera_pose.yaml' 


def main():
    print(f"--- Iniciando extração de {ARQUIVO_PKL_ENTRADA} ---")

    # Carregar o objeto Camera do arquivo Pickle 
    try:
        with open(ARQUIVO_PKL_ENTRADA, 'rb') as f_pkl:
            print("Carregando objeto 'Camera' do pickle...")
            camera_obj = pickle.load(f_pkl)
        
        print(f"Objeto 'Camera' carregado com sucesso.")
        print(f"  -> distortion_model: {camera_obj.distortion_model}")
        print(f"  -> resolution: {camera_obj.resolution}")

    except FileNotFoundError:
        print(f"ERRO: Arquivo de entrada '{ARQUIVO_PKL_ENTRADA}' não encontrado!", file=sys.stderr)
        sys.exit(1)
    except (NameError, ModuleNotFoundError) as e:
        print(f"ERRO: Não foi possível recriar a classe 'Camera'!", file=sys.stderr)
        print(f"  -> Detalhe: {e}", file=sys.stderr)
        print(f"  -> Certifique-se que o 'import' da sua classe 'Camera' está correto no topo deste script.", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"ERRO ao ler o arquivo pickle: {e}", file=sys.stderr)
        sys.exit(1)

    # self.pose é a matriz 4x4 T_mundo_<-_camera
    if not hasattr(camera_obj, 'pose'):
        print(f"ERRO: O objeto 'Camera' carregado não tem o atributo 'pose'.", file=sys.stderr)
        sys.exit(1)
        
    pose_matrix = camera_obj.pose 
    
    # Extrai a Matriz de Rotação 3x3 (R)
    R_matrix = pose_matrix[0:3, 0:3]
    
    # Extrai o Vetor de Translação 3x1 (t)
    t_vector = pose_matrix[0:3, 3]
    
    print("\nDados de Pose extraídos:")
    print("Matriz R (Rotação 3x3):\n", R_matrix)
    print("Vetor t (Translação 3x1):\n", t_vector)

    # --- Formatar os dados para o YAML do nó ROS ---
    # O nosso nó 'static_calib_publisher.py' espera:
    # translation: [x, y, z]
    # rotation_matrix: [R00, R01, R02, R10, R11, R12, R20, R21, R22] (lista achatada)

    # Converte numpy para listas Python
    t_list = t_vector.tolist()
    R_list_flat = R_matrix.tolist()

    # Monta o dicionário final no formato exato que o nó ROS espera
    dados_para_yaml = {
        'static_tf': {
            'target_frame': 'World', 
            'child_frame': 'camera',
            'translation': t_list,
            'rotation_matrix': R_list_flat
            }
        }
    
    print("\nDados formatados para o YAML:")
    print(dados_para_yaml)

    # Escrever o arquivo YAML
    try:
        with open(ARQUIVO_YAML_SAIDA, 'w') as f_yaml:
            print(f"\nEscrevendo arquivo YAML: {ARQUIVO_YAML_SAIDA}...")
            yaml.dump(dados_para_yaml, f_yaml, sort_keys=False, default_flow_style=False)
            
        print("--- Extração concluída com sucesso! ---")
        
    except Exception as e:
        print(f"ERRO ao escrever o arquivo YAML: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    main()