def make_circuit_solver(R1: float, R2: float, R3: float, Rn: float):
    """
    Returns f_voltages(I1, I2, I3) and g_currents(V1, V2, V3)
    for a three-channel circuit connected to a shared node.

    Each channel has a series resistor (R1, R2, R3), and the
    shared node is connected to ground through Rn.
    """

    def f_voltages(I1: float, I2: float, I3: float) -> tuple[float, float, float, float]:
        """Calculates V1, V2, V3, and Vn from target currents."""

        Vn = (I1 + I2 + I3) * Rn

        V1 = I1 * R1 + Vn
        V2 = I2 * R2 + Vn
        V3 = I3 * R3 + Vn

        return V1, V2, V3, Vn

    def g_currents(V1: float, V2: float, V3: float) -> tuple[float, float, float, float]:
        """Calculates I1, I2, I3, and Vn from input voltages."""

        conductance_sum = 1 / R1 + 1 / R2 + 1 / R3 + 1 / Rn

        Vn = (V1 / R1 + V2 / R2 + V3 / R3) / conductance_sum

        I1 = (V1 - Vn) / R1
        I2 = (V2 - Vn) / R2
        I3 = (V3 - Vn) / R3

        return I1, I2, I3, Vn

    return f_voltages, g_currents

if __name__ == "__main__":
    # 1. Define resistances (ohms)
    R1, R2, R3, Rn = 2000.0, 4000.0, 3000.0, 2000.0 # modify

    f_voltages, g_currents = make_circuit_solver(R1, R2, R3, Rn)

    print(
        f"Resistances: R1 = {R1:.1f} Ω, "
        f"R2 = {R2:.1f} Ω, R3 = {R3:.1f} Ω, "
        f"Rn = {Rn:.1f} Ω\n"
    )

    # 2. Define test cases (currents in amperes) # modify as needed
    test_cases = [
        ("Equal currents", 0.001, 0.001, 0.001),
        ("One channel off", 0.001, 0.000, 0.001),
        ("Unequal currents", 0.0005, 0.001, 0.0015),
        ("Low currents", 0.0001, 0.0002, 0.0003),
        ("One channel dominant", 0.002, 0.0005, 0.000),
    ]

    # 3. Run each test case
    for name, target_I1, target_I2, target_I3 in test_cases:
        print(f"\n--- {name} ---")

        # Calculate required input voltages
        v1, v2, v3, vn = f_voltages(
            target_I1, target_I2, target_I3
        )

        print(
            f"Target currents: I1 = {target_I1 * 1000:.2f} mA, "
            f"I2 = {target_I2 * 1000:.2f} mA, "
            f"I3 = {target_I3 * 1000:.2f} mA"
        )

        print(
            f"Required voltages: V1 = {v1:.2f} V, "
            f"V2 = {v2:.2f} V, V3 = {v3:.2f} V, "
            f"Vn = {vn:.2f} V"
        )

        if not all(v >= 0.0 for v in (v1, v2, v3)):
            print("WARNING: Negative input voltage!")
            continue

        # Verify currents using calculated voltages
        i1, i2, i3, vn_check = g_currents(v1, v2, v3)

        print(
            f"Verified currents: I1 = {i1 * 1000:.2f} mA, "
            f"I2 = {i2 * 1000:.2f} mA, "
            f"I3 = {i3 * 1000:.2f} mA"
        )
        print(f"Verified node voltage: Vn = {vn_check:.2f} V")
