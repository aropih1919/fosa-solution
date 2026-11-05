"""Contrôles communs de fraîcheur pour les mesures horodatées."""


def nanosecondes_stamp(stamp):
    """Convertit un `builtin_interfaces/Time` en nanosecondes entières."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def horodatage_recent(stamp_ns, maintenant_ns, age_max_sec):
    """Refuse les stamps nuls, futurs ou plus vieux que la limite configurée."""
    if stamp_ns <= 0 or maintenant_ns <= 0:
        return False
    age_ns = maintenant_ns - stamp_ns
    return 0 <= age_ns <= int(age_max_sec * 1_000_000_000)
