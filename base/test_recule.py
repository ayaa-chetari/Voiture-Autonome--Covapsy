from rpi_hardware_pwm import HardwarePWM
import time

PROP_REPOS = 7.36   # ton point mort étalonné

pwm_prop = HardwarePWM(pwm_channel=0, hz=50)
pwm_prop.start(PROP_REPOS)

def recule():
    # 1er click : freinage (zone arrière)
    print("1er click : freinage...")
    pwm_prop.change_duty_cycle(PROP_REPOS - 1.5)
    time.sleep(0.3)

    # retour au point mort obligatoire entre les deux clicks
    print("point mort...")
    pwm_prop.change_duty_cycle(PROP_REPOS)
    time.sleep(0.3)

    # 2ème click : recul s'engage maintenant
    print("2ème click : recul...")
    pwm_prop.change_duty_cycle(PROP_REPOS - 0.3)  # recul lent

try:
    recule()
    time.sleep(2)
    pwm_prop.change_duty_cycle(PROP_REPOS)
    print("Arrêt.")
except KeyboardInterrupt:
    pwm_prop.change_duty_cycle(PROP_REPOS)
finally:
    time.sleep(0.5)
    pwm_prop.stop()