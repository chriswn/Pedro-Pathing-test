package SubSystems.Scoring;

import java.util.Arrays;

public class PatternScorer {
    private ArtifactColor[] motif; // length 3
    private ArtifactColor[] pattern; // length 9

    public PatternScorer(ArtifactColor[] motif) {
        if (motif.length != 3) throw new IllegalArgumentException("Motif must have 3 colors");
        this.motif = motif;
        this.pattern = new ArtifactColor[9];
        for (int i = 0; i < 9; i++) {
            pattern[i] = motif[i % 3];
        }
    }

    // Returns number of pattern matches
    public int scorePattern(ArtifactColor[] rampArtifacts) {
        int score = 0;
        for (int i = 0; i < Math.min(rampArtifacts.length, 9); i++) {
            if (rampArtifacts[i] == pattern[i]) {
                score++;
            }
        }
        return score;
    }

    public ArtifactColor[] getPattern() {
        return pattern;
    }

    public static String patternString(ArtifactColor[] pattern) {
        StringBuilder sb = new StringBuilder();
        for (ArtifactColor c : pattern) {
            sb.append(c == ArtifactColor.GREEN ? "G" : "P");
        }
        return sb.toString();
    }
}

