import React, { useState, useCallback, useEffect } from 'react';
import styles from './styles.module.css';

/**
 * Quiz Component
 * Feature: 002-ui-improvements
 * Task: T070, T074, T075, T076, T078
 *
 * Inline quiz component with:
 * - Multiple choice questions
 * - Immediate feedback
 * - Score tracking
 * - Accessibility support
 */

const QuizContext = React.createContext();

/**
 * Main Quiz Component
 */
export default function Quiz({
  quizId,
  questions = [],
  showFeedback = true,
  showScore = true,
  allowRetry = true,
  shuffleOptions = false,
  passThreshold = 0.7
}) {
  const [currentQuestion, setCurrentQuestion] = useState(0);
  const [selectedAnswers, setSelectedAnswers] = useState({});
  const [showResults, setShowResults] = useState(false);
  const [hasSubmitted, setHasSubmitted] = useState(false);

  // Load saved progress
  useEffect(() => {
    if (typeof window === 'undefined') return;

    try {
      const saved = localStorage.getItem(`quiz-${quizId}`);
      if (saved) {
        const data = JSON.parse(saved);
        setSelectedAnswers(data.answers || {});
        if (data.completed) {
          setShowResults(true);
          setHasSubmitted(true);
        }
      }
    } catch (e) {
      console.warn('Could not load quiz progress:', e);
    }
  }, [quizId]);

  /**
   * Save progress to localStorage
   */
  const saveProgress = useCallback((answers, completed = false) => {
    if (typeof window === 'undefined') return;

    try {
      localStorage.setItem(`quiz-${quizId}`, JSON.stringify({
        answers,
        completed,
        timestamp: Date.now()
      }));
    } catch (e) {
      console.warn('Could not save quiz progress:', e);
    }
  }, [quizId]);

  /**
   * Handle answer selection
   */
  const handleSelectAnswer = useCallback((questionId, optionIndex) => {
    if (hasSubmitted) return;

    setSelectedAnswers(prev => {
      const newAnswers = {
        ...prev,
        [questionId]: optionIndex
      };
      saveProgress(newAnswers, showResults);
      return newAnswers;
    });
  }, [hasSubmitted, saveProgress, showResults]);

  /**
   * Submit quiz
   */
  const handleSubmit = useCallback(() => {
    setHasSubmitted(true);
    setShowResults(true);
    saveProgress(selectedAnswers, true);
  }, [selectedAnswers, saveProgress]);

  /**
   * Reset quiz
   */
  const handleReset = useCallback(() => {
    setSelectedAnswers({});
    setCurrentQuestion(0);
    setShowResults(false);
    setHasSubmitted(false);
    saveProgress({}, false);
  }, [saveProgress]);

  /**
   * Calculate score
   */
  const calculateScore = useCallback(() => {
    if (!questions.length) return { correct: 0, total: 0, percentage: 0 };

    let correct = 0;
    questions.forEach(q => {
      const selected = selectedAnswers[q.id];
      if (selected === q.correctAnswer) {
        correct++;
      }
    });

    return {
      correct,
      total: questions.length,
      percentage: correct / questions.length
    };
  }, [questions, selectedAnswers]);

  /**
   * Shuffle options if enabled.
   *
   * Always returns { option, originalIndex, isCorrect } objects: the
   * caller scores and selects by originalIndex, so shuffling the
   * display order can never remap an answer onto the wrong option.
   * The old code mapped the shuffled objects back to plain strings,
   * dropping the remap — with shuffle on, correct answers scored as
   * wrong. Shuffling is off by default, so this was dormant.
   */
  const getShuffledOptions = useCallback((options, correctIndex) => {
    const tagged = options.map((option, index) => ({
      option,
      originalIndex: index,
      isCorrect: index === correctIndex
    }));
    if (!shuffleOptions) return tagged;

    // Fisher-Yates shuffle
    for (let i = tagged.length - 1; i > 0; i--) {
      const j = Math.floor(Math.random() * (i + 1));
      [tagged[i], tagged[j]] = [tagged[j], tagged[i]];
    }

    return tagged;
  }, [shuffleOptions]);

  if (!questions.length) {
    return (
      <div className={styles.empty}>
        <p>No questions available for this quiz.</p>
      </div>
    );
  }

  const score = calculateScore();
  const passed = score.percentage >= passThreshold;

  return (
    <QuizContext.Provider
      value={{
        quizId,
        selectedAnswers,
        hasSubmitted,
        showResults,
        showFeedback,
        getShuffledOptions
      }}
    >
      <div
        className={styles.quiz}
        role="region"
        aria-label={`Quiz: ${quizId}`}
      >
        {showResults ? (
          <QuizResults
            score={score}
            passed={passed}
            questions={questions}
            onRetry={allowRetry ? handleReset : undefined}
          />
        ) : (
          <>
            <QuizProgress
              current={currentQuestion}
              total={questions.length}
              answered={Object.keys(selectedAnswers).length}
            />

            {questions.map((question, index) => (
              <QuizQuestion
                key={question.id || index}
                question={question}
                questionNumber={index + 1}
                isVisible={index === currentQuestion}
                onSelect={handleSelectAnswer}
              />
            ))}

            <QuizNavigation
              current={currentQuestion}
              total={questions.length}
              answered={Object.keys(selectedAnswers).length}
              onPrevious={() => setCurrentQuestion(Math.max(0, currentQuestion - 1))}
              onNext={() => setCurrentQuestion(Math.min(questions.length - 1, currentQuestion + 1))}
              onSubmit={handleSubmit}
              canSubmit={Object.keys(selectedAnswers).length === questions.length}
            />
          </>
        )}
      </div>
    </QuizContext.Provider>
  );
}

/**
 * Quiz Progress Indicator
 */
function QuizProgress({ current, total, answered }) {
  const progress = (answered / total) * 100;

  return (
    <div className={styles.progress}>
      <div className={styles.progressInfo}>
        <span className={styles.progressText}>
          Question {current + 1} of {total}
        </span>
        <span className={styles.progressText}>
          {answered} of {total} answered
        </span>
      </div>
      <div className={styles.progressBar}>
        <div
          className={styles.progressFill}
          style={{ width: `${progress}%` }}
          role="progressbar"
          aria-valuenow={answered}
          aria-valuemin={0}
          aria-valuemax={total}
          aria-label={`Quiz progress: ${answered} of ${total} questions answered`}
        />
      </div>
    </div>
  );
}

/**
 * Quiz Question Component
 */
function QuizQuestion({ question, questionNumber, isVisible, onSelect }) {
  const { selectedAnswers, hasSubmitted, showFeedback, getShuffledOptions } = React.useContext(QuizContext);
  const questionId = question.id || `q-${questionNumber}`;
  const selectedAnswer = selectedAnswers[questionId];
  const isCorrect = selectedAnswer === question.correctAnswer;

  if (!isVisible) return null;

  const options = getShuffledOptions(question.options, question.correctAnswer);

  return (
    <div className={styles.question}>
      {/* data-quiz-question + data-quiz-question-text: the translator
          collects the question text span on its own, so the number
          and difficulty badge (sibling spans) survive a translation
          instead of being wiped when the h4 is rewritten. */}
      <h4 className={styles.questionTitle} data-quiz-question>
        <span className={styles.questionNumber}>{questionNumber}.</span>
        <span data-quiz-question-text>{question.question}</span>
        {question.difficulty && (
          <span className={styles.difficulty} aria-label={`Difficulty: ${question.difficulty}`}>
            {question.difficulty}
          </span>
        )}
      </h4>

      {/* The explanation is shown only after submitting, in the
          feedback block below — showing it here, above the options
          before answering, gives the answer away. */}

      <div
        className={styles.options}
        role="radiogroup"
        aria-label={`Question ${questionNumber} options`}
      >
        {options.map((item) => {
          const option = item.option;
          const originalIndex = item.originalIndex;
          const optionId = `${questionId}-option-${originalIndex}`;
          const isSelected = selectedAnswer === originalIndex;
          const showCorrect = hasSubmitted && item.isCorrect;
          const showIncorrect = hasSubmitted && isSelected && !isCorrect;

          return (
            <label
              key={optionId}
              className={[
                styles.option,
                isSelected && styles.selected,
                showCorrect && styles.correct,
                showIncorrect && styles.incorrect,
                hasSubmitted && styles.disabled
              ].filter(Boolean).join(' ')}
            >
              <input
                type="radio"
                name={questionId}
                value={originalIndex}
                checked={isSelected}
                onChange={() => onSelect(questionId, originalIndex)}
                disabled={hasSubmitted}
                className={styles.optionInput}
                aria-describedby={
                  showFeedback && (showCorrect || showIncorrect)
                    ? `${optionId}-feedback`
                    : undefined
                }
              />
              <span className={styles.optionText}>{option}</span>
              {(showCorrect || showIncorrect) && showFeedback && (
                <span
                  id={`${optionId}-feedback`}
                  className={styles.feedbackIcon}
                  aria-label={showCorrect ? 'Correct answer' : 'Incorrect answer'}
                >
                  {showCorrect ? '✓' : '✗'}
                </span>
              )}
            </label>
          );
        })}
      </div>

      {/* No explanation is shown here before answering (that was the
          A3 leak). After submit the questions are replaced by the
          results review below, which is where each explanation
          appears — this per-question block can never render after
          submit, so the review is the only place it lives. */}
    </div>
  );
}

/**
 * Quiz Navigation
 */
function QuizNavigation({ current, total, answered, onPrevious, onNext, canSubmit, onSubmit }) {
  const { hasSubmitted } = React.useContext(QuizContext);

  return (
    <div className={styles.navigation}>
      <button
        onClick={onPrevious}
        disabled={current === 0}
        className={styles.navButton}
        aria-label="Previous question"
      >
        ← Previous
      </button>

      {!hasSubmitted && (
        <button
          onClick={canSubmit ? onSubmit : onNext}
          disabled={!canSubmit && current === total - 1}
          className={styles.primaryButton}
          aria-label={canSubmit ? 'Submit quiz' : 'Next question'}
        >
          {canSubmit ? 'Submit Quiz' : current === total - 1 ? 'Review' : 'Next →'}
        </button>
      )}

      {hasSubmitted && current === total - 1 && (
        <span className={styles.completed}>Quiz completed</span>
      )}
    </div>
  );
}

/**
 * Quiz Results Component
 */
function QuizResults({ score, passed, questions, onRetry }) {
  const { selectedAnswers, showFeedback } = React.useContext(QuizContext);

  return (
    <div className={styles.results}>
      <div className={`${styles.scoreBanner} ${passed ? styles.passed : styles.failed}`}>
        <div className={styles.scoreIcon}>{passed ? '🎉' : '📚'}</div>
        <h3 className={styles.scoreTitle}>
          {passed ? 'Congratulations!' : 'Keep Learning!'}
        </h3>
        <p className={styles.scoreText}>
          You scored {score.correct} out of {score.total} ({Math.round(score.percentage * 100)}%)
        </p>
        {passed ? (
          <p className={styles.scoreMessage}>You have passed this quiz!</p>
        ) : (
          <p className={styles.scoreMessage}>
            Review the material and try again to improve your score.
          </p>
        )}
      </div>

      <div className={styles.review}>
        <h4>Question Review</h4>
        {questions.map((question, index) => {
          const questionId = question.id || `q-${index + 1}`;
          const selected = selectedAnswers[questionId];
          const isCorrect = selected === question.correctAnswer;
          const selectedText =
            selected !== undefined && question.options[selected] !== undefined
              ? question.options[selected]
              : null;

          return (
            <div
              key={questionId}
              className={`${styles.reviewItem} ${isCorrect ? styles.correct : styles.incorrect}`}
            >
              <div className={styles.reviewHeader}>
                <span className={styles.reviewIcon}>{isCorrect ? '✓' : '✗'}</span>
                <span className={styles.reviewQuestion}>
                  {index + 1}. {question.question}
                </span>
              </div>
              {/* The explanation lives here, in the post-submit review:
                  submitting swaps the questions for these results, so
                  the per-question feedback block can never render after
                  submit. Nothing here shows before answering. */}
              {showFeedback && (
                <>
                  {selectedText !== null && (
                    <p className={styles.reviewAnswer}>
                      Your answer: {selectedText}
                    </p>
                  )}
                  {selectedText === null && (
                    <p className={styles.reviewAnswer}>No answer given.</p>
                  )}
                  {!isCorrect && question.correctAnswer !== undefined && (
                    <p className={styles.reviewAnswer}>
                      Correct answer: {question.options[question.correctAnswer]}
                    </p>
                  )}
                  {question.explanation && (
                    <p className={styles.reviewExplanation}>{question.explanation}</p>
                  )}
                </>
              )}
            </div>
          );
        })}
      </div>

      {onRetry && !passed && (
        <button onClick={onRetry} className={styles.retryButton}>
          Try Again
        </button>
      )}
    </div>
  );
}

/**
 * Export quiz data helper
 */
export { QuizContext };
