import React from "react";

import {
  Canvas,
  Circle,
  Image,
  useClock,
  useImage,
} from "@shopify/react-native-skia";
import { View } from "react-native";
import { useDerivedValue } from "react-native-reanimated";

export const PulseIndicator = () => {
  const clock1 = useClock();
  const expo = useImage(require("./img/expo.png"));
  const heart = useImage(require("./img/heart.png"));

  const interval = 1250;

  const scale = useDerivedValue(() => {
    return ((clock1.value % interval) / interval) * 130;
  }, [clock1]);

  const opacity = useDerivedValue(() => {
    return 0.9 - (clock1.value % interval) / interval;
  }, [clock1]);

  const scale2 = useDerivedValue(() => {
    return (((clock1.value + 400) % interval) / interval) * 130;
  }, [clock1]);

  const opacity2 = useDerivedValue(() => {
    return 0.9 - ((clock1.value + 400) % interval) / interval;
  }, [clock1]);

  if (!expo || !heart) {
    return <View />;
  }

  return (
    <Canvas style={{ height: 300, width: 300 }}>
      <Circle cx={150} cy={150} r={50} opacity={1} color="#FF6060"></Circle>
      <Circle cx={150} cy={150} r={scale} opacity={opacity} color="#FF6060" />
      <Circle cx={150} cy={150} r={scale2} opacity={opacity2} color="#FF6060" />
      <Image
        image={expo}
        fit="contain"
        x={125}
        y={125}
        width={50}
        height={50}
      />
    </Canvas>
  );
};
