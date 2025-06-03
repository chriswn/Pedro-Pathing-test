package SubSystems.Scoring;

import java.util.Arrays;

public class PatternScorerTest {
    public static void runTest(String motifStr, String rampStr, boolean gateClosed) {
        ArtifactColor[] motif = new ArtifactColor[3];
        for (int i = 0; i < 3; i++) motif[i] = ArtifactColor.fromChar(motifStr.charAt(i));
        PatternScorer scorer = new PatternScorer(motif);
        ArtifactColor[] ramp = new ArtifactColor[rampStr.length()];
        for (int i = 0; i < rampStr.length(); i++) ramp[i] = ArtifactColor.fromChar(rampStr.charAt(i));
        int patternScore = gateClosed ? scorer.scorePattern(ramp) : 0;
        System.out.println("Motif:   " + PatternScorer.patternString(motif));
        System.out.println("Pattern: " + PatternScorer.patternString(scorer.getPattern()));
        System.out.println("Ramp:    " + PatternScorer.patternString(ramp));
        System.out.println("Gate Closed: " + gateClosed);
        System.out.println("Pattern Score: " + patternScore + " / 9\n");
    }

    public static void main(String[] args) {
        // Test all motifs and edge cases
        System.out.println("=== PATTERN SCORING TESTS ===\n");
        // Motif GPP (ID 21), perfect match
        runTest("GPP", "GPPGPPGPP", true);
        // Motif PGP (ID 22), perfect match
        runTest("PGP", "PGPPGPPGP", true);
        // Motif PPG (ID 23), perfect match
        runTest("PPG", "PPGPPGPPG", true);
        // Motif GPP, mismatches
        runTest("GPP", "GPPGPGGPP", true);
        // Motif GPP, ramp shorter than 9
        runTest("GPP", "GPPGPP", true);
        // Motif GPP, gate open (should score 0)
        runTest("GPP", "GPPGPPGPP", false);
    }
}
