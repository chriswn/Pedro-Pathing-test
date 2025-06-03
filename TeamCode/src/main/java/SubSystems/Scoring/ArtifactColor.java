package SubSystems.Scoring;

public enum ArtifactColor {
    GREEN, PURPLE;

    public static ArtifactColor fromChar(char c) {
        switch (Character.toUpperCase(c)) {
            case 'G': return GREEN;
            case 'P': return PURPLE;
            default: throw new IllegalArgumentException("Invalid artifact color: " + c);
        }
    }
}

