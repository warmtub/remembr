# Changelog

## [0.1.4] - 2025-03-24
### Add
- web recorder
### Change
- refine asr_node to asr_node_lite, do asr only (audio bytes to string)

## [0.1.3] - 2025-03-13
### Add
- improve caption promte and max token size
- evaluation topic in memory build phase
- change video topic
- position retreive from memory
- evaluation scripts
### Change
- skip prompt keyword

## [0.1.2] - 2025-01-08
### Add
- terminate topic
- remembr_status topic
- goal_text topic
### Change
- use (x, y, z) in prompt example
- fix string input for retrieve_from_position
- split AIMessage
- new query interrupt previous one
- break infinite generate loop after three retries

## [0.1.1] - 2024-12-20
### Change
- test on amr
- fix caption pose timestamp

## [0.1.0] - 2024-12-18
### Added
- webcam and fake pose node
- add caption pose publisher in caption node
### Change
- fix common_utils bug
