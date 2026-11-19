/** Verify the isolated raw-forward, legacy-base and legacy-reverse experiment. */
public class Tree2AblationProbe {
  public static void main(String[] args) {
    int[] previousLeft = {-100, 0, 100};
    for (int i = 0; i < args.length; i += 2) {
      float angle = Float.parseFloat(args[i]);
      float expected = Float.parseFloat(args[i + 1]);
      for (int left : previousLeft) {
        int right = -left;
        float forward = TreeModels.difference(angle, 1, left, right);
        float reverse = TreeModels.difference(angle, -1, left, right);
        if (forward != expected || TreeModels.forwardSpeedScale(angle, forward) != 1f
            || reverse != TreeModels.legacyDifference(angle, -1, left, right)) {
          throw new AssertionError("B policy differs at angle " + angle + ", previous left " + left);
        }
      }
    }
  }
}
