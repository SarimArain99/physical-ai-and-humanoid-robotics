/**
 * Quiz Embed Component for MDX
 * Feature: 002-ui-improvements
 * Task: T077
 *
 * Usage in MDX:
 * ```jsx
 * import Quiz from '@site/src/client-components/QuizEmbed';
 *
 * <Quiz id="ros2-intro" />
 * ```
 */

import React from 'react';
import Quiz from '@site/src/components/Quiz';
import { getQuiz } from '@site/src/data/quizzes';

export default function QuizEmbed({ id, ...props }) {
  const quizData = getQuiz(id);

  if (!quizData) {
    return (
      <div style={{
        padding: '2rem',
        background: 'var(--color-bg-surface)',
        border: '1px solid var(--color-error)',
        borderRadius: 'var(--radius-lg)',
        color: 'var(--color-error)',
        textAlign: 'center'
      }}>
        Quiz not found: {id}
      </div>
    );
  }

  return <Quiz quizId={id} questions={quizData.questions} {...props} />;
}
