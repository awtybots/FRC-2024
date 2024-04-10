package frc.robot.commands.Positions;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.ArmConstants;
import java.util.Optional;
import java.util.stream.DoubleStream;
import java.util.stream.IntStream;
import java.util.stream.Stream;
// import org.junit.jupiter.api.DisplayName;
// import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

class SpeakerShotTest {

  // @Test // IDK why this is here on second thought this test doesn't make sense
  // @DisplayName("ReasonableA,B,C Values")
  // void reasonableValues() {
  //   assertTrue(
  //       -ArmConstants.QUADRATIC_B * Math.pow((2 * ArmConstants.QUADRATIC_A), -1) > 0); // -b/2a
  // }

  @ParameterizedTest(name = "Known Points Are Valid Points")
  @MethodSource("knownPointsProvider")
  void knownPointsCheck(double speakerDistance, double expectedResult) {
    double reasonableError = 0.01;
    double calculatedValue =
        SpeakerShot.getRequiredAngle(Optional.of(speakerDistance)).get().doubleValue();
    assertEquals(
        expectedResult,
        calculatedValue,
        reasonableError,
        () -> calculatedValue + " should equal " + expectedResult);
  }

  /** Returns a stream of arguments of known (distance, angle) pairs. */
  static Stream<Arguments> knownPointsProvider() {
    return Stream.of(
        Arguments.of(1.00, ArmConstants.ShootCloseAngle),
        Arguments.of(1.7112, ArmConstants.ShootMediumAngle)); // ,
    // Arguments.of(2.794, 0.922), // Fake numbers
    // Arguments.of(3, 0.89), // Fake numbers
    // Arguments.of(4, 0.51)); // Fake numbers
  }

  @ParameterizedTest(name = "Inside Allowed Range")
  @MethodSource("speakerRangeProvider")
  void InsideAllowedRange(double doubleStream) {
    double calculatedAngle =
        ArmConstants.QUADRATIC_A * doubleStream * doubleStream
            + ArmConstants.QUADRATIC_B * doubleStream
            + ArmConstants.QUADRATIC_C;
    assertTrue(
        Math.abs(calculatedAngle) < ArmConstants.UpwardsAngle
            && Math.abs(calculatedAngle) > ArmConstants.FloorPickupAngle);
  }

  /**
   * Returns a range of double values that represent possible realistic-ish distances from the
   * speaker
   */
  static DoubleStream speakerRangeProvider() {
    double maxPossibleRange;

    if (Math.pow(ArmConstants.QUADRATIC_B, 2)
            - 4 * ArmConstants.QUADRATIC_A * ArmConstants.QUADRATIC_C
        > 0) {
      double solution1 =
          (-ArmConstants.QUADRATIC_B
                  - Math.sqrt(
                      Math.pow(ArmConstants.QUADRATIC_B, 2)
                          - 4 * ArmConstants.QUADRATIC_A * ArmConstants.QUADRATIC_C))
              * Math.pow(2 * ArmConstants.QUADRATIC_A, -1);
      double solution2 =
          (-ArmConstants.QUADRATIC_B
                  + Math.sqrt(
                      Math.pow(ArmConstants.QUADRATIC_B, 2)
                          - 4 * ArmConstants.QUADRATIC_A * ArmConstants.QUADRATIC_C))
              * Math.pow(2 * ArmConstants.QUADRATIC_A, -1);
      maxPossibleRange = Math.max(solution1, solution2);
    } else {
      double solution1 =
          (-ArmConstants.QUADRATIC_B
                  - Math.sqrt(
                      Math.pow(ArmConstants.QUADRATIC_B, 2)
                          - 4
                              * ArmConstants.QUADRATIC_A
                              * (ArmConstants.QUADRATIC_C - ArmConstants.UpwardsAngle)))
              * Math.pow(2 * ArmConstants.QUADRATIC_A, -1);
      double solution2 =
          (-ArmConstants.QUADRATIC_B
                  + Math.sqrt(
                      Math.pow(ArmConstants.QUADRATIC_B, 2)
                          - 4
                              * ArmConstants.QUADRATIC_A
                              * (ArmConstants.QUADRATIC_C - ArmConstants.UpwardsAngle)))
              * Math.pow(2 * ArmConstants.QUADRATIC_A, -1);
      maxPossibleRange = Math.max(solution1, solution2);
    }

    IntStream intStream = IntStream.range(2, (int) maxPossibleRange * 10);
    DoubleStream doubleStream =
        intStream.mapToDouble(
            (number) -> {
              System.out.println(((double) (number)) / 10);
              return (((double) (number)) / 10);
            });
    return doubleStream;
  }
}
