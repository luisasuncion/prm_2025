from setuptools import find_packages, setup
from glob import glob
import os

# Comandos para compilar o workspace:
# Entrar na raíz do workspace: cd ~/ros2_ws
# Compila o workspace: colcon build --symlink-install --packages-select prm
# Não esqueça de dar source no workpace para atualizar os novos pacotes no terminal
# se você configurou o source no .bashrc, você pode fechar e abrir novamente o
# terminal após compilar o workspce.

def package_dir_tree(target_dir, base_install_path):
    """
    Recursively collects all files in target_dir and maps them
    to their corresponding install paths under base_install_path.
    """
    entries = {}
    for filepath in glob(os.path.join(target_dir, '**'), recursive=True):
        if os.path.isfile(filepath):
            relpath = os.path.relpath(filepath, start=target_dir)
            install_path = os.path.join(base_install_path, os.path.dirname(relpath))
            entries.setdefault(install_path, []).append(filepath)
    return list(entries.items())

package_name = 'prm'

data_files = [
    # Requerido pelo ROS2
    ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
    ('share/' + package_name, ['package.xml']),

    # Adicionado para atender as demandas do nosso pacote!
    (f'share/{package_name}/launch', glob('launch/*.py')),
    (f'share/{package_name}/description', glob('description/*.urdf.xacro')),
    (f'share/{package_name}/rviz', glob('rviz/*.rviz')),
    (f'share/{package_name}/config', glob('config/*.yaml')),
    (f'share/{package_name}/world', glob('world/*.sdf')),
]

# Adiciona todos os arquivos de modelos da pasta models/ recursivamente (se existir)
if os.path.isdir('models'):
    data_files.extend(package_dir_tree('models', f'share/{package_name}/models'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matheus Machado',
    maintainer_email='matheus.m.santos@icmc.usp.br',
    description='Pacote da disciplina SSC0712: Programação de Robôs Móveis',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tartaruga = prm.controle_tartaruga:main',
            'controle_robo = prm.controle_robo:main',
            'detecta_bandeira = prm.detecta_bandeira:main',
        ],
    },
)