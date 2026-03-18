import multi_vehicle_scheduler.scheduler_backend as backend


def main():
    a = 3
    b = 4
    print(f"Adding {a} and {b} gives {backend.add(a, b)}")


if __name__ == "__main__":
    main()
