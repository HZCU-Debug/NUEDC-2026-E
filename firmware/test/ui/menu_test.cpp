#include <cassert>
#include <string>
#include <vector>

#include "runtime/program_runner.h"
#include "ui/menu.h"

class RecordingProgram final : public runtime::Program {
public:
    RecordingProgram()
        : starts(0), updates(0), stops(0), event(ui::Event::None) {}

    void start(Adafruit_GFX&, runtime::SystemState&) override { ++starts; }

    void update(Adafruit_GFX&, runtime::SystemState&, ui::Event next) override {
        ++updates;
        event = next;
    }

    void stop(runtime::SystemState&) override { ++stops; }

    int starts;
    int updates;
    int stops;
    ui::Event event;
};

struct Configuration {
    explicit Configuration(int value) : value(value) {}

    int value;
};

class ConfiguredProgram final : public runtime::Program {
public:
    ConfiguredProgram() : configuredValue(0), startedValue(0) {}

    void configure(const Configuration& configuration) {
        configuredValue = configuration.value;
    }

    void start(Adafruit_GFX&, runtime::SystemState&) override {
        startedValue = configuredValue;
    }

    void update(Adafruit_GFX&, runtime::SystemState&, ui::Event) override {}

    int configuredValue;
    int startedValue;
};

class DeferredExitProgram final : public runtime::Program {
public:
    DeferredExitProgram() : exits(0), stops(0), ready(false) {}

    void start(Adafruit_GFX&, runtime::SystemState&) override {}

    void update(Adafruit_GFX&, runtime::SystemState&, ui::Event) override {}

    void requestExit() override { ++exits; }

    bool readyToExit() const override { return ready; }

    void stop(runtime::SystemState&) override { ++stops; }

    int exits;
    int stops;
    bool ready;
};

int main() {
    Adafruit_GFX display(240, 135);
    runtime::SystemState state;
    runtime::ProgramRunner runner(state);
    RecordingProgram firstProgram;
    RecordingProgram secondProgram;
    RecordingProgram thirdProgram;
    ui::Item first("First", runner, firstProgram);
    ui::Item second("Second", runner, secondProgram);
    ui::Item third("Third", runner, thirdProgram);
    ui::Item* items[] = {&first, &second, &third};
    ui::Menu menu(display, "Test Menu", items, 3);

    assert(menu.begin());
    assert(display.printed ==
           std::vector<std::string>({"Test Menu", "First", "Second", "Third"}));
    assert(menu.begin());

    menu.loop(ui::Event::Up);
    menu.loop(ui::Event::Select);
    assert(thirdProgram.starts == 1);

    menu.loop(ui::Event::Down);
    assert(thirdProgram.updates == 1);
    assert(thirdProgram.event == ui::Event::Down);

    menu.loop(ui::Event::Back);
    assert(thirdProgram.stops == 1);

    menu.loop(ui::Event::Down);
    menu.loop(ui::Event::Select);
    assert(firstProgram.starts == 1);

    menu.loop(ui::Event::Back);
    ConfiguredProgram configured;
    ui::ConfiguredItem<ConfiguredProgram, Configuration> configuredFirst(
        "Configured First", runner, configured, Configuration(1));
    ui::ConfiguredItem<ConfiguredProgram, Configuration> configuredSecond(
        "Configured Second", runner, configured, Configuration(2));
    ui::Item* configuredItems[] = {&configuredFirst, &configuredSecond};
    ui::Menu configuredMenu(display, "Configured", configuredItems, 2);

    assert(configuredMenu.begin());
    configuredMenu.loop(ui::Event::Select);
    assert(configured.startedValue == 1);
    configuredMenu.loop(ui::Event::Back);
    configuredMenu.loop(ui::Event::Down);
    configuredMenu.loop(ui::Event::Select);
    assert(configured.startedValue == 2);

    runtime::SystemState deferredState;
    runtime::ProgramRunner deferredRunner(deferredState);
    DeferredExitProgram deferredProgram;
    ui::Item deferredItem("Deferred", deferredRunner, deferredProgram);
    ui::Item* deferredItems[] = {&deferredItem};
    ui::Menu deferredMenu(display, "Deferred", deferredItems, 1);

    assert(deferredMenu.begin());
    deferredMenu.loop(ui::Event::Select);
    deferredMenu.loop(ui::Event::Back);
    assert(deferredProgram.exits == 1);
    assert(deferredProgram.stops == 0);

    deferredProgram.ready = true;
    deferredMenu.loop(ui::Event::None);
    assert(deferredProgram.stops == 1);

    return 0;
}
