package frc.robot.commands.Positions;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.ArmConstants;
import java.util.Optional;
import java.util.stream.DoubleStream;
import java.util.stream.IntStream;
import java.util.stream.Stream;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

class SpeakerShotTest {

  @Test
  @DisplayName("ReasonableA,B,C Values")
  void reasonableValues() {
    assertTrue(
        -ArmConstants.QUADRATIC_B * Math.pow((2 * ArmConstants.QUADRATIC_A), -1) > 0); // -b/2a
  }

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
        Arguments.of(1.27, ArmConstants.ShootCloseAngle),
        Arguments.of(2.032, ArmConstants.ShootMediumAngle),
        Arguments.of(2.794, 0.922), // Fake numbers
        Arguments.of(3, 0.89), // Fake numbers
        Arguments.of(4, 0.51)); // Fake numbers
  }

  @ParameterizedTest(name = "Inside Allowed Range")
  @MethodSource("speakerRangeProvider")
  void InsideAllowedRange(double doubleStream) {
    assertTrue(
        Math.abs(doubleStream) < ArmConstants.UpwardsAngle
            && Math.abs(doubleStream) > ArmConstants.FloorPickupAngle);
  }

  /**
   * Returns a range of double values that represent possible realistic-ish distances from the
   * speaker
   */
  static DoubleStream speakerRangeProvider() {
    getMaxPossibleRange();
    IntStream intStream = IntStream.range(1, (int) getMaxPossibleRange() + 1);
    DoubleStream doubleStream =
        intStream.mapToDouble(
            (number) -> {
              System.out.println(((double) (number)) / 8);
              return ((double) (number)) / 8;
            });
    return doubleStream;
  }

  /** Solves the quadratic equation for the larger number */
  static double getMaxPossibleRange() {
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
    return Math.max(solution1, solution2);
  }
}
