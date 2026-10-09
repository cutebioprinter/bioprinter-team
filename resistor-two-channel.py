def make_circuit_solver(R1: float, R2: float, Rn: float):
    """
    Returns f_voltages(I1, I2) and g_currents(V1, V2) for the parallel circuit.
    """
    def f_voltages(I1: float, I2: float) -> tuple[float, float, float]:
        """Calculates V1, V2, and Vn given desired target currents I1, I2."""
        Vn = (I1 + I2) * Rn
        V1 = (I1 * R1) + Vn
        V2 = (I2 * R2) + Vn
        return V1, V2, Vn

    def g_currents(V1: float, V2: float) -> tuple[float, float, float]:
        """Calculates I1, I2, and Vn given input voltages V1, V2."""
        Vn = (V1 / R1 + V2 / R2) / (1 / R1 + 1 / R2 + 1 / Rn)
        I1 = (V1 - Vn) / R1
        I2 = (V2 - Vn) / R2
        return I1, I2, Vn

    return f_voltages, g_currents


if __name__ == "__main__":
    R1, R2, Rn = 40.0, 10.0, 10.0
    f_voltages, g_currents = make_circuit_solver(R1, R2, Rn)

    print(f"Resistance: R1 = {R1:.1f} Ω, R2 = {R2:.1f} Ω, Rn = {Rn:.1f} Ω\n")

    # 1. Define target currents
    target_I1 = 0.05   #  100 mA 
    target_I2 = 0.05   #  -80 mA

    # 2. Compute required voltages
    v1, v2, vn = f_voltages(target_I1, target_I2)
    print(f"Inputs:  I1 = {target_I1*1000:.1f} mA, I2 = {target_I2*1000:.1f} mA")
    print(f"Yields:  V1 = {v1:.2f} V, V2 = {v2:.2f} V, Vn = {vn:.2f} V\n")

    # 3. Verify currents using calculated voltages
    i1_check, i2_check, vn_check = g_currents(v1, v2)
    print(f"Checking g(V1={v1:.2f}V, V2={v2:.2f}V):")
    print(f"Result:  I1 = {i1_check*1000:.1f} mA, I2 = {i2_check*1000:.1f} mA")