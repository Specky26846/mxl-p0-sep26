# P0::GPSComponent

NEO M9N component

## Introduction

<!-- High level introduction, FPP interfaces?, high-level features? -->

## GPS Data Availability

The component publishes zero latitude, longitude, altitude, ground speed, and satellite count whenever a poll fails or no NAV-PVT packet is returned, so the channels remain visible in the GDS. After five consecutive missed polls, it emits `GpsDataUnavailable` once. The event is emitted again only after GPS data resumes and a later outage reaches the threshold.

## Requirements

| Name | Description | Rationale | Validation |
|---|---|---|---|
|   |   |   |   |

## Design

<!-- Explain high level design and important internal details -->

## Configuration

<!-- If the component requires configuration at initialization, document here -->
