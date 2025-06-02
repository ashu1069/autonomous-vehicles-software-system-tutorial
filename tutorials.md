# Tutorial-I: Sensors

> This tutorial excerpt explains the crucial **preprocessing step** for sensor data in autonomous driving systems. It highlights two key functions of this stage: **data format abstraction**, which ensures compatibility with sensors from various manufacturers by converting diverse outputs into a unified format, and **common data processing**, which applies fundamental algorithms to refine raw sensor data by addressing noise and other imperfections. These preprocessing steps are essential for providing reliable information about the vehicle's position, orientation, and velocity, which is foundational for higher-level autonomous driving software modules.
>

Okay, let's dive into a crucial topic for understanding how self-driving cars work: **sensor preprocessing**. Imagine you're building a self-driving car – a complex system that needs to understand the world around it accurately and reliably. The very foundation of this understanding comes from its sensors.

*So, what are sensors in this context?* Think of cameras, lidar, radar, and systems that figure out the car's position and movement like GNSS/INS. These sensors collect a huge amount of raw data, but **this raw data isn't immediately ready for the car's "brain" (the software stack) to use**. This is where the preprocessing stage comes in.

The preprocessing stage is absolutely critical. Its main goal is to ensure that the higher-level modules of the autonomous driving system receive information that is **clean, reliable, and in a usable format**. Without this step, building a robust AV software stack would be incredibly difficult, if not impossible. Let's break down its two main roles:

### Abstraction of data formats

This is a really important concept for achieving **modularity and vendor independence**. Think about integrating sensors from different companies; each one might output data differently – maybe different units, different ways of structuring the information, or even different data encoding. The preprocessing layer acts like a universal translator. It takes these diverse streams of raw data and converts them all into a **consistent, standardized format** that the rest of the autonomous driving software stack can easily understand and use.

**Why is this important?**

This abstraction means that the algorithms that make decisions (like planning a path or detecting objects) don't need to be tightly tied to a specific brand or type of sensor. This makes the system much **more flexible**, **easier to maintain**, and allows for **upgrading sensors** with different options down the line.

### Performing common/primitive sensor data processing

As mentioned earlier, raw sensor data isn't perfect. It often contains things like **noise** (random errors), **biases** (systematic errors), and might not be in a format that's directly useful for tasks like figuring out where obstacles are or precisely locating the car. The preprocessing stage applies **fundamental algorithms** to refine this data. These are tasks that are generally applicable regardless of what the system will do with the data later. It's about getting the data into the best possible shape before it's used for more complex tasks.

---

To give you a sense of how specific this processing can be, the sources point to separate designs for preprocessing different types of sensor data. Click on the buttons below to explore them:

- [GNSS/INS Data Pre-processing Design](gnss-and-ins.md){: .btn }
- [Image Pre-processing Design](image.md){: .btn }
- [Point Cloud Pre-processing Design](lidar.md){: .btn }
- [Radar Objects Data Pre-processing Design](radar.md){: .btn }
- [Synchronization](synchronization.md){: .btn }

---

*In essence, the preprocessing stage is the **unsung hero** of the autonomous driving software stack. By standardizing data formats and cleaning up raw sensor information, it provides the **clean and reliable foundation** that all subsequent steps depend on. Without this crucial stage, achieving accurate perception, localization, and ultimately, safe autonomous driving, would be significantly more challenging.*

NOTE:

The material is inspired from the [Autoware](https://autowarefoundation.github.io/autoware-documentation/main/) documentation and some figures are directly copied.

## Author

Ashutosh Kumar

Website**: [https://ashu1069.github.io/](https://ashu1069.github.io/)**