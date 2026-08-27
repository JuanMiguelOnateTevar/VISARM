import math

# Parámetros geométricos, medidas en mm de las diferentes partes del robot
H_SHOULDER = 122.5
L1 = 150.0
L2 = 155.0
# Límites GRBL, angulo maximo que puede llegar las articulaciones
COMMAND_LIMITS = {
    "X": (0.0, 84.0),
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
# Offset del TCP respecto al centro de la muñeca
# cuando el gripper está horizontal
TOOL_OFFSET_R = 96.0   # mm hacia delante
TOOL_OFFSET_Z = 45.0   # mm hacia abajo

def inverse_kinematics(
    x: float,
    y: float,
    z: float
) -> dict[str, float]:
    """
    Cinemática inversa.

    Convierte la posición cartesiana (x, y, z) del TCP
    en los ángulos matemáticos de las articulaciones [rad].

    El gripper se mantiene siempre paralelo al suelo.
    """

    # -----------------------
    # BASE
    # -----------------------

    ang_base = math.atan2(y, x)


    # -----------------------
    # TCP -> MUÑECA
    # -----------------------

    # Distancia radial del TCP respecto al eje de la base
    r_tcp = math.sqrt(x*x + y*y)

    # Como el TCP está 96 mm por delante de la muñeca
    r_wrist = r_tcp - TOOL_OFFSET_R

    # Como el TCP está 45 mm por debajo de la muñeca
    z_wrist = z + TOOL_OFFSET_Z


    # -----------------------
    # MUÑECA -> SHOULDER FRAME
    # -----------------------

    z_shoulder = z_wrist - H_SHOULDER

    d = math.sqrt(
        r_wrist*r_wrist
        + z_shoulder*z_shoulder
    )


    # -----------------------
    # ALCANCE
    # -----------------------

    if not abs(L1 - L2) <= d <= L1 + L2:
        raise ValueError(
            "Posición fuera del alcance del robot"
        )


    # -----------------------
    # ELBOW
    # -----------------------
    cos_elbow = (
        d*d - L1*L1 - L2*L2) / (2 * L1 * L2)

    cos_elbow = max(
        -1.0,
        min(1.0, cos_elbow)
    )

    # Rama compatible con tu robot físico
    ang_elbow = -math.acos(cos_elbow)


    # -----------------------
    # SHOULDER
    # -----------------------

    alpha = math.atan2(
        z_shoulder,
        r_wrist
    )

    beta = math.atan2(
        L2 * math.sin(ang_elbow),
        L1 + L2 * math.cos(ang_elbow)
    )

    ang_shoulder = alpha - beta


    # -----------------------
    # WRIST
    # -----------------------

    # Queremos que el gripper permanezca horizontal.
    #
    # tool_angle =
    # shoulder + elbow + wrist - 90°
    #
    # tool_angle = 0°
    #
    # wrist = 90° - shoulder - elbow

    ang_wrist = (
        math.radians(85.0)
        - ang_shoulder
        - ang_elbow
    )


    return {
        "base": ang_base,
        "shoulder": ang_shoulder,
        "elbow": ang_elbow,
        "wrist": ang_wrist,
    }
    
def forward_kinematics(
    ang_shoulder: float,
    ang_elbow: float,
    ang_base: float,
    ang_wrist: float,
) -> dict[str, float]:
    """
    Cinemática directa.
    Convierte los ángulos matemáticos de las articulaciones
    [rad] en la posición cartesiana del TCP [mm].
    Se asume que el gripper está paralelo al suelo.
    """

    # Posición de la muñeca en el plano r-z
    r_wrist = (L1 * math.cos(ang_shoulder)+ L2 * math.cos(
            ang_shoulder + ang_elbow))

    z_wrist = (H_SHOULDER + L1 * math.sin(ang_shoulder)
        + L2 * math.sin(ang_shoulder + ang_elbow))

    # Offset muñeca -> TCP
    r_tcp = r_wrist + TOOL_OFFSET_R
    z_tcp = z_wrist - TOOL_OFFSET_Z

    # Proyección del radio al plano XY
    x = r_tcp * math.cos(ang_base)
    y = r_tcp * math.sin(ang_base)

    return {
        "x": x,
        "y": y,
        "z": z_tcp,
    }
    
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