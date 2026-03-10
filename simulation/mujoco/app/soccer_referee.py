# SPDX-FileCopyrightText: Copyright (c) MOS-Brain Contributors
# SPDX-License-Identifier: GPL-3.0-or-later

from __future__ import annotations

from dataclasses import dataclass


class PlayMode:
    BEFORE_KICK_OFF = "BeforeKickOff"
    KICK_OFF_LEFT = "KickOff_Left"
    KICK_OFF_RIGHT = "KickOff_Right"
    PLAY_ON = "PlayOn"
    THROW_IN_LEFT = "KickIn_Left"
    THROW_IN_RIGHT = "KickIn_Right"
    CORNER_KICK_LEFT = "corner_kick_left"
    CORNER_KICK_RIGHT = "corner_kick_right"
    GOAL_KICK_LEFT = "goal_kick_left"
    GOAL_KICK_RIGHT = "goal_kick_right"
    GOAL_LEFT = "Goal_Left"
    GOAL_RIGHT = "Goal_Right"
    FREE_KICK_LEFT = "free_kick_left"
    FREE_KICK_RIGHT = "free_kick_right"
    GAME_OVER = "GameOver"


@dataclass
class RefereeRules:
    half_time: float = 10.0 * 60.0
    kick_off_time: float = 15.0
    throw_in_time: float = 15.0
    corner_kick_time: float = 15.0
    goal_kick_time: float = 15.0
    free_kick_time: float = 15.0
    goal_pause_time: float = 3.0


class MujocoSoccerReferee:
    def __init__(
        self,
        field_length: float,
        field_width: float,
        goal_width: float,
        goal_height: float,
        goalie_area_depth: float = 1.0,
        goalie_area_extra_width: float = 1.2,
        *,
        rules: RefereeRules | None = None,
        red_count: int = 0,
        blue_count: int = 0,
    ):
        self.field_length = float(field_length)
        self.field_width = float(field_width)
        self.goal_width = float(goal_width)
        self.goal_height = float(goal_height)
        self.goalie_area_depth = float(goalie_area_depth)
        self.goalie_area_width = float(goal_width + 2.0 * goalie_area_extra_width)
        self.rules = rules if rules is not None else RefereeRules()
        self.red_count = int(red_count)
        self.blue_count = int(blue_count)

        self.play_time = 0.0
        self.play_mode = PlayMode.BEFORE_KICK_OFF
        self.play_mode_started_at = 0.0
        self.left_score = 0
        self.right_score = 0

        self.agent_na_touch_ball: int | None = None
        self.team_na_score: str | None = None
        self._did_act = False

        self.ball_place_pos: tuple[float, float] | None = None
        self._ball_last_contact: int | None = None

        self.reset()

    def reset(self):
        self.play_time = 0.0
        self.play_mode = PlayMode.BEFORE_KICK_OFF
        self.play_mode_started_at = 0.0
        self.left_score = 0
        self.right_score = 0
        self.agent_na_touch_ball = None
        self.team_na_score = None
        self._did_act = False
        self.ball_place_pos = None
        self._ball_last_contact = None
        self.kick_off("left")

    @staticmethod
    def _team_from_rid(rid: int | None) -> str | None:
        if rid is None:
            return None
        return "left" if rid < 7 else "right"

    @staticmethod
    def _opponent(team: str) -> str:
        return "right" if team == "left" else "left"

    def _set_play_mode(self, play_mode: str):
        self.play_mode = play_mode
        self.play_mode_started_at = self.play_time

    def _set_team_mode(self, team: str, left_mode: str, right_mode: str):
        self._set_play_mode(left_mode if team == "left" else right_mode)

    def kick_off(self, team: str):
        self._did_act = True
        self._set_team_mode(team, PlayMode.KICK_OFF_LEFT, PlayMode.KICK_OFF_RIGHT)
        self.agent_na_touch_ball = None
        self.team_na_score = team
        self.ball_place_pos = (0.0, 0.0)

    def play_on(self):
        self._did_act = True
        self._set_play_mode(PlayMode.PLAY_ON)

    def throw_in(self, team: str, ball_x: float, ball_y: float):
        self._did_act = True
        self._set_team_mode(team, PlayMode.THROW_IN_LEFT, PlayMode.THROW_IN_RIGHT)
        self.agent_na_touch_ball = None
        self.team_na_score = None
        half_wid = 0.5 * self.field_width
        y = -half_wid if ball_y < 0 else half_wid
        self.ball_place_pos = (float(ball_x), float(y))

    def corner_kick(self, team: str, ball_y: float):
        self._did_act = True
        self._set_team_mode(team, PlayMode.CORNER_KICK_LEFT, PlayMode.CORNER_KICK_RIGHT)
        self.agent_na_touch_ball = None
        self.team_na_score = None
        half_len = 0.5 * self.field_length
        half_wid = 0.5 * self.field_width
        x = half_len if team == "left" else -half_len
        y = -half_wid if ball_y < 0 else half_wid
        self.ball_place_pos = (float(x), float(y))

    def goal_kick(self, team: str):
        self._did_act = True
        self._set_team_mode(team, PlayMode.GOAL_KICK_LEFT, PlayMode.GOAL_KICK_RIGHT)
        self.agent_na_touch_ball = None
        self.team_na_score = None
        half_len = 0.5 * self.field_length
        x = -half_len + 0.5 * self.goalie_area_depth if team == "left" else half_len - 0.5 * self.goalie_area_depth
        self.ball_place_pos = (float(x), 0.0)

    def free_kick(self, team: str, ball_x: float, ball_y: float):
        self._did_act = True
        self._set_team_mode(team, PlayMode.FREE_KICK_LEFT, PlayMode.FREE_KICK_RIGHT)
        self.agent_na_touch_ball = None
        self.team_na_score = team
        self.ball_place_pos = (float(ball_x), float(ball_y))

    def goal(self, team: str):
        self._did_act = True
        if team == "left":
            self.left_score += 1
            self._set_play_mode(PlayMode.GOAL_LEFT)
        else:
            self.right_score += 1
            self._set_play_mode(PlayMode.GOAL_RIGHT)
        self.agent_na_touch_ball = None
        self.team_na_score = None

    def game_over(self):
        self._did_act = True
        self._set_play_mode(PlayMode.GAME_OVER)
        self.agent_na_touch_ball = None
        self.team_na_score = None

    def _play_mode_age(self) -> float:
        return self.play_time - self.play_mode_started_at

    def _is_ball_in_left_goal(self, x: float, y: float, z: float) -> bool:
        return x <= -0.5 * self.field_length and abs(y) <= 0.5 * self.goal_width and z <= self.goal_height

    def _is_ball_in_right_goal(self, x: float, y: float, z: float) -> bool:
        return x >= 0.5 * self.field_length and abs(y) <= 0.5 * self.goal_width and z <= self.goal_height

    def _in_left_goalie_area(self, x: float, y: float) -> bool:
        half_len = 0.5 * self.field_length
        return (-half_len <= x <= -half_len + self.goalie_area_depth) and (abs(y) <= 0.5 * self.goalie_area_width)

    def _in_right_goalie_area(self, x: float, y: float) -> bool:
        half_len = 0.5 * self.field_length
        return (half_len - self.goalie_area_depth <= x <= half_len) and (abs(y) <= 0.5 * self.goalie_area_width)

    def _check_fouls(self, active_contact: int | None):
        if self.team_na_score is not None and active_contact is not None and self._ball_last_contact is not None:
            self.team_na_score = None

        if self.agent_na_touch_ball is not None and active_contact is not None and self._ball_last_contact is not None:
            if self.agent_na_touch_ball == self._ball_last_contact and self.agent_na_touch_ball == active_contact:
                team = self._team_from_rid(active_contact)
                if team is not None:
                    self.free_kick(self._opponent(team), self._last_ball_pos[0], self._last_ball_pos[1])
                    return
            self.agent_na_touch_ball = None

    def _check_timeouts(self):
        if self._did_act:
            return
        pm = self.play_mode
        if pm == PlayMode.PLAY_ON:
            return
        age = self._play_mode_age()
        if pm in (PlayMode.KICK_OFF_LEFT, PlayMode.KICK_OFF_RIGHT) and age > self.rules.kick_off_time:
            self.play_on()
            return
        if pm in (PlayMode.THROW_IN_LEFT, PlayMode.THROW_IN_RIGHT) and age > self.rules.throw_in_time:
            self.play_on()
            return
        if pm in (PlayMode.CORNER_KICK_LEFT, PlayMode.CORNER_KICK_RIGHT) and age > self.rules.corner_kick_time:
            self.play_on()
            return
        if pm in (PlayMode.FREE_KICK_LEFT, PlayMode.FREE_KICK_RIGHT) and age > self.rules.free_kick_time:
            self.play_on()
            return
        if pm == PlayMode.GOAL_KICK_LEFT and age > self.rules.goal_kick_time:
            self.play_on()
            return
        if pm == PlayMode.GOAL_KICK_RIGHT and age > self.rules.goal_kick_time:
            self.play_on()
            return
        if pm == PlayMode.GOAL_LEFT and age > self.rules.goal_pause_time:
            self.kick_off("right")
            return
        if pm == PlayMode.GOAL_RIGHT and age > self.rules.goal_pause_time:
            self.kick_off("left")
            return

    def _check_location_triggers(self, ball_x: float, ball_y: float, ball_z: float):
        if self._did_act:
            return
        pm = self.play_mode
        if pm in (PlayMode.GOAL_LEFT, PlayMode.GOAL_RIGHT):
            return

        if self._is_ball_in_left_goal(ball_x, ball_y, ball_z):
            if pm == PlayMode.GOAL_KICK_LEFT:
                self.play_on()
            else:
                self.goal("right")
            return

        if self._is_ball_in_right_goal(ball_x, ball_y, ball_z):
            if pm == PlayMode.GOAL_KICK_RIGHT:
                self.play_on()
            else:
                self.goal("left")
            return

        half_len = 0.5 * self.field_length
        half_wid = 0.5 * self.field_width
        if abs(ball_x) > half_len or abs(ball_y) > half_wid:
            last_team = self._team_from_rid(self._ball_last_contact)
            if ball_x < -half_len:
                if last_team == "left":
                    self.corner_kick("right", ball_y)
                else:
                    self.goal_kick("left")
            elif ball_x > half_len:
                if last_team == "right":
                    self.corner_kick("left", ball_y)
                else:
                    self.goal_kick("right")
            else:
                if last_team is None:
                    self.throw_in("left", ball_x, ball_y)
                else:
                    self.throw_in(self._opponent(last_team), ball_x, ball_y)
            return

        if pm == PlayMode.GOAL_KICK_LEFT and (not self._in_left_goalie_area(ball_x, ball_y)):
            self.play_on()
            return
        if pm == PlayMode.GOAL_KICK_RIGHT and (not self._in_right_goalie_area(ball_x, ball_y)):
            self.play_on()
            return

    def _check_contact_triggers(self, active_contact: int | None):
        if self._did_act:
            return
        if active_contact is None:
            return
        pm = self.play_mode
        if pm == PlayMode.PLAY_ON:
            return
        if pm in (
            PlayMode.KICK_OFF_LEFT,
            PlayMode.KICK_OFF_RIGHT,
            PlayMode.THROW_IN_LEFT,
            PlayMode.THROW_IN_RIGHT,
            PlayMode.CORNER_KICK_LEFT,
            PlayMode.CORNER_KICK_RIGHT,
            PlayMode.FREE_KICK_LEFT,
            PlayMode.FREE_KICK_RIGHT,
        ):
            team = self._team_from_rid(active_contact)
            if team == "left" and self.red_count > 1:
                self.agent_na_touch_ball = active_contact
            elif team == "right" and self.blue_count > 1:
                self.agent_na_touch_ball = active_contact
            self.play_on()

    def update(self, dt: float, ball_x: float, ball_y: float, ball_z: float, active_contact: int | None):
        self._did_act = False
        self._last_ball_pos = (float(ball_x), float(ball_y), float(ball_z))

        if self.play_mode not in (PlayMode.BEFORE_KICK_OFF, PlayMode.GAME_OVER):
            self.play_time += max(0.0, float(dt))

        if self.play_time >= self.rules.half_time and self.play_mode != PlayMode.GAME_OVER:
            self.game_over()
            return

        if active_contact is not None and active_contact != self._ball_last_contact:
            self._ball_last_contact = active_contact

        self._check_fouls(active_contact)
        self._check_timeouts()
        self._check_location_triggers(ball_x, ball_y, ball_z)
        self._check_contact_triggers(active_contact)

    def consume_ball_place(self) -> tuple[float, float] | None:
        out = self.ball_place_pos
        self.ball_place_pos = None
        return out

    def game_state_dict(self) -> dict:
        return {
            "play_mode": self.play_mode,
            "play_time": self.play_time,
            "score_left": self.left_score,
            "score_right": self.right_score,
            "team_na_score": self.team_na_score,
            "scoring_allowed": "both",
        }
