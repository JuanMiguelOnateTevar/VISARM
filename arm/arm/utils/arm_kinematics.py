import math

# Parámetros geométricos, medidas en mm de las diferentes partes del robot
H_SHOULDER = 122.5
L1 = 150.0
L2 = 155.0
# Límites GRBL, angulo maximo que puede llegar las articulaciones
COMMAND_LIMITS = {
    "X": (0.0, 68.0),
    "Y": (0.0, 128.0),
    "Z": (0.0, 228.0),
    "B": (0.0, 158.0),
}
# Calibración entre los ángulos del modelo matemático y los valores de comando de GRBL
CALIBRATION = {
    'shoulder':{'axis': 'X', 'command_ref': 7.0, 'model_ref_deg': 90.0, 'sign': -1.0, 'scale': 1.0},
    'elbow':{'axis': 'Y', 'command_ref': 128.0, 'model_ref_deg': 0.0, 'sign': 1.0, 'scale': 1.0}, #dudo con sign -1.0
    'base':{'axis': 'Z', 'command_ref': 43.0, 'model_ref_deg': 0.0, 'sign': 1.0, 'scale': 1.0},
    'wrist':{'axis': 'B', 'command_ref': 157.0, 'model_ref_deg': 0.0, 'sign': -1.0, 'scale': 1.0} #dudo con sign 1.0
    }

def inverse_kinematics(x: float, y: float, z: float)-> dict[str, float]:
    '''
    Cinematica inversa convertimos una posicion cartesiana (x, y, z) del efector fian a 
    angulos en radianes para las diferentes articulaciones del robot. 
    '''
    #Angulo para la base
    ang_base = math.atan2(y, x)

    #Angulo para el codo
    r = math.sqrt(x*x + y*y)
    z_shoulder = z - H_SHOULDER
    d = math.sqrt(r*r + z_shoulder*z_shoulder)
    if not(abs(L1-L2) <= d <= L1+L2):
        raise ValueError(f'Posición fuera del alcance del robot')
    cos_elbow = (d*d - L1*L1 - L2*L2)/(2*L1*L2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow)) # Protección frente a pequeños errores numéricos
    ang_elbow = -math.acos(cos_elbow)

    #Angulo para el hombro
    alpha = math.atan2(z_shoulder, r)
    beta = math.atan2(L2*math.sin(ang_elbow), L1+L2*math.cos(ang_elbow))
    ang_shoulder = alpha - beta

    #Angulo para la muñeca
    ang_wrist = 0.0

    return {"base": ang_base, "shoulder": ang_shoulder, "elbow": ang_elbow, "wrist": ang_wrist}

    
def forward_kinematics(ang_shoulder: float, ang_elbow: float, ang_base: float, ang_wrist: float) -> dict[str, float]:
    '''
    Cinematica directa este convierte angulos en radianes de las articulaciones en una posicion
    cartisiana (x, y, z) del efector final
    '''
    #Distancia en horizontal desde la base 'r'
    r = L1*math.cos(ang_shoulder)+L2*math.cos(ang_shoulder+ang_elbow)
    #Posición en x
    x = r*math.cos(ang_base)
    #Posición en y
    y = r*math.sin(ang_base)
    #Posición en z
    z = H_SHOULDER+L1*math.sin(ang_shoulder)+L2*math.sin(ang_shoulder+ang_elbow)
    return {"x": x, "y": y, "z": z}
    
def check_point(x: float, y: float, z: float) -> dict[str, object]:
    '''
    Valida que el codigo matematico tenga sentido, simplemente compara resultados entre cinematica directa e inversa.
    '''
    target = {"x": x, "y": y, "z": z}
    tolerance = 0.01
    angles = inverse_kinematics(x=x, y=y, z=z)
    calcule_position = forward_kinematics(ang_base=angles.get('base'), ang_shoulder=angles.get('shoulder'), ang_elbow=angles.get('elbow'), ang_wrist=angles.get('wrist'))

    for name, value in calcule_position.items():
        error = abs(value - target[name])
        if error > tolerance:
            return {'status': False, 'message': 'No se hizo correctamente la cinematica inversa', 'error': error, 'target': target, 'angles': angles, 'calcule_position': calcule_position}
        status = True
    
    if status == True:
        return {'status': True, 'message': 'Se hizo correctamente la cinematica inversa', 'error': error, 'target': target, 'angles': angles, 'calcule_position': calcule_position}

def model_joints_to_grbl_targets(joints: dict[str, float]) -> dict[str, float]:
    '''
    Transforma ángulos matemáticos del modelo, expresados en
    radianes, en objetivos de comando GRBL.
    '''
    grbl_targets = {}
    for art, value in CALIBRATION.items():
        grbl_targets[value['axis']] = value['command_ref']+value['sign']*value['scale']*(math.degrees(joints[art])-value['model_ref_deg'])

    return grbl_targets

def grbl_targets_to_model_joints(joints: dict[str, float]) -> dict[str, float]:
    '''
    radianes, en objetivos de comando GRBL,
    a ángulos matemáticos del modelo, expresados en
    radianes.
    '''
    model_joints = {}
    for art, value in CALIBRATION.items():
        model_joints[art] = math.radians(((joints[art]-value['command_ref'])/(value['sign']*value['scale']))+value['model_ref_deg'])

    return model_joints

def build_g1_command(grbl_targets: dict[str, float], feed_rate: float) -> str:
    '''
    Mensaje a mandar al brazo.
    '''
    for key, value in COMMAND_LIMITS.items():
        if grbl_targets[key] < value[0] or grbl_targets[key] > value[1]:
            raise ValueError(f'Valor de {key} = {grbl_targets[key]:.3f} fuera de rango {value}')

    if feed_rate <= 0:
        raise ValueError('El fedd_rate tiene que ser mayor a 0')
    return (
            f"G1 "
            f"X{grbl_targets['X']:.3f} "
            f"Y{grbl_targets['Y']:.3f} "
            f"Z{grbl_targets['Z']:.3f} "
            f"B{grbl_targets['B']:.3f} "
            f"F{feed_rate:.3f}"
        )

if __name__ == "__main__":
    target_xyz = {
        "x": 170.0,
        "y": 0.0,
        "z": 100.0,
    }

    joints = inverse_kinematics(
        x=target_xyz["x"],
        y=target_xyz["y"],
        z=target_xyz["z"],
    )

    print("IK:")
    for name, value in joints.items():
        print(name, math.degrees(value))

    grbl_targets = model_joints_to_grbl_targets(joints)

    print("GRBL targets:")
    print(grbl_targets)

    message = build_g1_command(
        grbl_targets=grbl_targets,
        feed_rate=250.0,
    )

    print(message)
        # for name, angle_rad in joints.items():
    #     print(
    #         name,
    #         angle_rad,
    #         math.degrees(angle_rad),
    #     )

    # positiones = forward_kinematics(
    #     ang_base=joints["base"],
    #     ang_shoulder=joints["shoulder"],
    #     ang_elbow=joints["elbow"],
    #     ang_wrist=joints["wrist"]
    # )
    # for name, position in positiones.items():
    #     print(
    #         name,
    #         position
    #     )
    # result = check_point(
    #     x=200.0,
    #     y=100.0,
    #     z=170.0,
    # )

    # if result.get('status') == True:
    #     print(result.get('message'))
    #     for name, angle in result.get('angles').items():
    #         print(
    #             name,
    #             angle,
    #             math.degrees(angle),
    #         )
    # else:
    #     message = result.get('message')
    #     print(f'ERROR: {message}')