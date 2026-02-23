package org.firstinspires.ftc.teamcode.global;

import org.firstinspires.ftc.teamcode.util.InterpolatedLookupTable;

public class OuttakeInterpLUTs {
    public InterpolatedLookupTable velocityLUT = new InterpolatedLookupTable();
    public InterpolatedLookupTable angleLUT = new InterpolatedLookupTable();
        // These are dummy values, replace with actual data after testing
    public OuttakeInterpLUTs() {
        velocityLUT.add(0, 0);
        velocityLUT.add(1, 1500);
        angleLUT.add(0, 0);
        angleLUT.add(1, 1);
    }
}
