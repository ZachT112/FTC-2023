package org.firstinspires.ftc.teamcode.util;

public class SimpleMath {
    private SimpleMath() {}

    public static int Sum(int[] nums) {
        int sum = 0;
        for (int i = 0; i < nums.length; i++) {
            sum += nums[i];
        }
        return sum;
    }

    public static int Average(int[] nums) {
        return Sum(nums) / nums.length;
    }
}
