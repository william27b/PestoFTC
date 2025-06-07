package com.shprobotics.pestocore.processing;

/*
Controls thinking, planning, problem-solving, and movement
 */

import android.content.Context;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.message.MessageCache;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.shprobotics.pestocore.drivebases.controllers.DriveController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import dalvik.system.DexFile;

public class FrontalLobe {
    @FunctionalInterface
    public interface Macro {
        Object run();
    }

    private static Map<String, Macro> macros;
    public static DriveController driveController;
    public static TeleOpController teleOpController;
    public static DeterministicTracker tracker;

    public static PestoTelemetry pestoTelemetry;

    private static List<Class<?>> configurations;

    private static void addConfigClasses() {
        if (configurations == null)
            configurations = new ArrayList<>();

        /*
        I took this code from FtcDashboard

        Thanks guys!!!
        */

        Set<String> IGNORED_PACKAGES = new HashSet<>(Arrays.asList(
                "java",
                "android",
                "com.sun",
                "com.vuforia",
                "com.google",
                "kotlin"
        ));

        ClassLoader classLoader = FtcDashboard.class.getClassLoader();

        Context context = AppUtil.getInstance().getApplication();
        try {
            DexFile dexFile = new DexFile(context.getPackageCodePath());

            List<String> classNames = Collections.list(dexFile.entries());

            for (String className : classNames) {
                boolean skip = false;
                for (String prefix : IGNORED_PACKAGES) {
                    if (className.startsWith(prefix)) {
                        skip = true;
                        break;
                    }
                }

                if (skip) {
                    continue;
                }

                try {
                    Class<?> configClass = Class.forName(className, false, classLoader);

                    if (!configClass.isAnnotationPresent(PestoConfig.class)
                            || configClass.isAnnotationPresent(Disabled.class)) {
                        continue;
                    }

                    if (!(ConfigInterface.class.isAssignableFrom(configClass)))
                        continue;

                    configurations.add(configClass);
                } catch (ClassNotFoundException | NoClassDefFoundError ignored) {
                    // dash is unable to access many classes and reporting every instance
                    // only clutters the logs
                }
            }
        } catch (IOException e) {
            RobotLog.logStackTrace(e);
        }
    }

    public static void initialize(HardwareMap hardwareMap) {
        macros = new HashMap<>();

        FrontalLobe.addConfigClasses();
        for (Class<?> configClass: configurations) {
            try {
                if (ConfigInterface.class.isAssignableFrom(configClass)) {
                    Method method = configClass.getMethod("initialize", HardwareMap.class);
                    method.invoke(null, hardwareMap);
                }
            } catch (NoSuchMethodException | IllegalAccessException | InvocationTargetException ignored) {

            }
        }

        MessageCache.initialize();

        pestoTelemetry = new PestoTelemetry();
        pestoTelemetry.clearAll();
        pestoTelemetry.update();
    }

    public static void addMacro(String alias, Macro macro) {
        if (macro == null)
            throw new MacroException("Macro cannot be null");

        if (alias == null || alias.isEmpty())
            throw new MacroException("Alias cannot be null");

        if (macros.containsKey(alias))
            throw new MacroException("Alias already assigned");

        macros.put(alias, macro);
    }

    public static Object useMacro(String alias) {
        if (!macros.containsKey(alias))
            throw new MacroException();

        Macro macro = macros.get(alias);

        if (macro == null)
            throw new MacroException("Macro cannot be null");

        return macro.run();
    }
}
