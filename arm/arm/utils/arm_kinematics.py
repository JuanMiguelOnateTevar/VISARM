import math

def inverse_kinematics(x: float, y: float, z: float)-> dict[str, float]:
    h_shoulder = 122.5
    l1 = 150.0
    l2 = 155.0

    #Angulo para la base
    ang_base = math.atan2(y, x)

    #Angulo para el codo
    r = math.sqrt(x*x + y*y)
    z_shoulder = z - h_shoulder
    d = math.sqrt(r*r + z_shoulder*z_shoulder)
    if not(abs(l1-l2) <= d <= l1+l2):
        raise ValueError(f'Posición fuera del alcance del robot')
    cos_elbow = (d*d - l1*l1 - l2*l2)/(2*l1*l2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow)) # Protección frente a pequeños errores numéricos
    ang_elbow = math.acos(cos_elbow)

    #Angulo para el hombro
    alpha = math.atan2(z_shoulder, r)
    beta = math.atan2(l2*math.sin(ang_elbow), l1+l2*math.cos(ang_elbow))
    ang_shoulder = alpha - beta

    #Angulo para la muñeca
    ang_wrist = 0.0

    return {"base": ang_base, "shoulder": ang_shoulder, "elbow": ang_elbow, "wrist": ang_wrist}

    
def forward_kinematics(ang_base: float, ang_shoulder: float, ang_elbow: float, ang_wrist: float) -> dict[str, float]:
    h_shoulder = 122.5
    l1 = 150.0
    l2 = 155.0
    #Distancia en horizontal desde la base 'r'
    r = l1*math.cos(ang_shoulder)+l2*math.cos(ang_shoulder+ang_elbow)
    #Posición en x
    x = r*math.cos(ang_base)
    #Posición en y
    y = r*math.sin(ang_base)
    #Posición en z
    z = h_shoulder+l1*math.sin(ang_shoulder)+l2*math.sin(ang_shoulder+ang_elbow)
    return {"x": x, "y": y, "z": z}
    
def check_point(x: float, y: float, z: float) -> dict[str, object]:
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

def build_g1_command(joints: dict[str, float], feed_rate: float) -> str:
    return (
        f"G1 "
        f"X{math.degrees(joints.get('shoulder')):.3f} "
        f"Y{math.degrees(joints.get('elbow')):.3f} "
        f"Z{math.degrees(joints.get('base')):.3f} "
        f"B{math.degrees(joints.get('wrist')):.3f} "
        f"F{feed_rate:.3f}"
    )

if __name__ == "__main__":
    joints = inverse_kinematics(
        x=200.0,
        y=100.0,
        z=170.0,
    )
    message = build_g1_command(joints=joints, feed_rate=150.0)
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